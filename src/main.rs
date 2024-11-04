use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin};
use bevy_fps_counter::FpsCounterPlugin;
use std::collections::HashMap;
use urdf_rs::{Geometry, Pose};

mod world;
use world::WorldPlugin;

mod camera;
use camera::CameraPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            bevy_stl::StlPlugin,
            FpsCounterPlugin,
            WorldPlugin,
            CameraPlugin,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
            EguiPlugin,
        ))
        .add_systems(
            Startup,
            (
                spawn_robots,
                (process_urdf_visuals, process_urdf_collisions),
            )
                .chain(),
        )
        .add_systems(Update, joint_control_ui)
        .run();
}

#[derive(Component)]
struct Robot;

#[derive(Component)]
struct RobotPart;

#[derive(Component)]
struct UrdfVisual {
    link_name: String,
    geometry: Geometry,
    material: Option<urdf_rs::Material>,
    origin: Pose,
}

#[derive(Component)]
struct URDFCollision {
    link_name: String,
    geometry: Geometry,
    origin: Pose,
}

#[derive(Component)]
struct JointComponent {
    name: String,
    joint_type: urdf_rs::JointType,
    axis: [f64; 3],
    current_position: f32,
}

fn spawn_robots(mut commands: Commands) {
    spawn_robot(
        &mut commands,
        Transform {
            translation: Vec3::new(0.0, 0.0, 0.0),
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            scale: Vec3::ONE,
        },
    );
}

fn spawn_robot(commands: &mut Commands, base_transform: Transform) {
    let urdf_path = "sample_description/urdf/low_cost_robot.urdf";
    let robot = urdf_rs::read_file(urdf_path).expect("Failed to read URDF file");

    let mut link_entities = HashMap::new();

    for link in &robot.links {
        let entity = commands
            .spawn((
                RobotPart,
                Name::new(link.name.clone()),
                TransformBundle::default(),
                VisibilityBundle::default(),
            ))
            .id();

        link_entities.insert(link.name.clone(), entity);
    }

    for joint in &robot.joints {
        let parent_link_name = &joint.parent.link;
        let child_link_name = &joint.child.link;

        let parent_entity = link_entities.get(parent_link_name).unwrap();
        let child_entity = link_entities.get(child_link_name).unwrap();

        let joint_origin_transform = urdf_to_transform(&joint.origin, &None);

        commands.entity(*child_entity).insert((
            JointComponent {
                name: joint.name.clone(),
                joint_type: joint.joint_type.clone(),
                axis: *joint.axis.xyz,
                current_position: 0.0,
            },
            TransformBundle::from_transform(Transform::IDENTITY),
        ));

        let joint_entity = commands
            .spawn((
                Name::new(format!("Joint: {}", joint.name)),
                TransformBundle::from_transform(joint_origin_transform),
                VisibilityBundle::default(),
            ))
            .id();

        commands.entity(joint_entity).add_child(*child_entity);
        commands.entity(*parent_entity).add_child(joint_entity);
    }

    let root_link_name = &robot.links[0].name;
    let root_link_entity = link_entities.get(root_link_name).unwrap();

    commands
        .spawn((
            Robot,
            Name::new(robot.name.clone()),
            TransformBundle::from_transform(Transform::IDENTITY),
            VisibilityBundle::default(),
        ))
        .add_child(*root_link_entity);

    commands
        .entity(*root_link_entity)
        .insert(TransformBundle::from_transform(base_transform));

    for link in &robot.links {
        let link_entity = link_entities.get(&link.name).unwrap();

        for visual in &link.visual {
            commands.entity(*link_entity).with_children(|parent| {
                parent.spawn((UrdfVisual {
                    link_name: link.name.clone(),
                    geometry: visual.geometry.clone(),
                    material: visual.material.clone(),
                    origin: visual.origin.clone(),
                },));
            });
        }

        for collision in &link.collision {
            commands.entity(*link_entity).with_children(|parent| {
                parent.spawn((URDFCollision {
                    link_name: link.name.clone(),
                    geometry: collision.geometry.clone(),
                    origin: collision.origin.clone(),
                },));
            });
        }
    }
}

fn process_urdf_visuals(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    query: Query<(Entity, &UrdfVisual), Added<UrdfVisual>>,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (entity, urdf_visual) in query.iter() {
        let (mesh_handle, material_handle) = match &urdf_visual.geometry {
            Geometry::Mesh { filename, .. } => {
                let mesh_handle = asset_server.load(filename);
                let material_handle = create_material(&urdf_visual.material, &mut materials);
                (mesh_handle, material_handle)
            }
            Geometry::Box { size } => {
                let mesh = Mesh::from(Cuboid::new(size[0] as f32, size[1] as f32, size[2] as f32));
                let mesh_handle = meshes.add(mesh);
                let material_handle = create_material(&urdf_visual.material, &mut materials);
                (mesh_handle, material_handle)
            }
            Geometry::Cylinder { radius, length } => {
                let mesh = Mesh::from(Cylinder {
                    radius: *radius as f32,
                    half_height: (*length as f32) / 2.0,
                });
                let mesh_handle = meshes.add(mesh);
                let material_handle = create_material(&urdf_visual.material, &mut materials);
                (mesh_handle, material_handle)
            }
            Geometry::Sphere { radius } => {
                let mesh = Mesh::from(Sphere {
                    radius: *radius as f32,
                });
                let mesh_handle = meshes.add(mesh);
                let material_handle = create_material(&urdf_visual.material, &mut materials);
                (mesh_handle, material_handle)
            }
            _ => {
                warn!("Unsupported geometry type: {:?}", urdf_visual.geometry);
                continue;
            }
        };

        let transform = urdf_to_transform(&urdf_visual.origin, &Some(urdf_visual.geometry.clone()));

        commands.entity(entity).insert((PbrBundle {
            mesh: mesh_handle,
            material: material_handle,
            transform,
            ..Default::default()
        },));
    }
}

fn process_urdf_collisions(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    query: Query<(Entity, &URDFCollision), Added<URDFCollision>>,
    asset_server: Res<AssetServer>,
) {
    for (entity, urdf_collision) in query.iter() {
        let mesh_handle = match &urdf_collision.geometry {
            Geometry::Mesh { filename, .. } => asset_server.load(filename),
            Geometry::Box { size } => {
                let mesh = Mesh::from(Cuboid::new(size[0] as f32, size[1] as f32, size[2] as f32));
                meshes.add(mesh)
            }
            Geometry::Cylinder { radius, length } => {
                let mesh = Mesh::from(Cylinder {
                    radius: *radius as f32,
                    half_height: (*length as f32) / 2.0,
                });
                meshes.add(mesh)
            }
            Geometry::Sphere { radius } => {
                let mesh = Mesh::from(Sphere {
                    radius: *radius as f32,
                });
                meshes.add(mesh)
            }
            _ => {
                warn!("Unsupported geometry type: {:?}", urdf_collision.geometry);
                continue;
            }
        };

        let transform = urdf_to_transform(
            &urdf_collision.origin,
            &Some(urdf_collision.geometry.clone()),
        );
        let body_type = match urdf_collision.link_name.as_str() {
            "base_link" => RigidBody::Static,
            _ => RigidBody::Kinematic,
        };

        commands.entity(entity).insert((
            (ColliderConstructor::ConvexDecompositionFromMesh, body_type),
            mesh_handle,
            transform,
        ));
    }
}

fn create_material(
    urdf_material: &Option<urdf_rs::Material>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> Handle<StandardMaterial> {
    let color = if let Some(material) = urdf_material {
        if let Some(urdf_color) = &material.color {
            Color::srgba(
                urdf_color.rgba[0] as f32,
                urdf_color.rgba[1] as f32,
                urdf_color.rgba[2] as f32,
                urdf_color.rgba[3] as f32,
            )
        } else {
            Color::srgba(0.8, 0.8, 0.8, 1.0)
        }
    } else {
        Color::srgba(0.8, 0.8, 0.8, 1.0)
    };

    materials.add(StandardMaterial {
        base_color: color,
        metallic: 0.7,
        ..Default::default()
    })
}

fn urdf_to_transform(origin: &Pose, geometry: &Option<Geometry>) -> Transform {
    let pos = origin.xyz;
    let rot = origin.rpy;

    let mut scale = Vec3::ONE;

    if let Some(Geometry::Mesh {
        scale: Some(mesh_scale),
        ..
    }) = geometry
    {
        scale = Vec3::new(
            mesh_scale[0] as f32,
            mesh_scale[1] as f32,
            mesh_scale[2] as f32,
        );
    }

    Transform {
        translation: Vec3::new(pos[0] as f32, pos[1] as f32, pos[2] as f32),
        rotation: Quat::from_euler(EulerRot::XYZ, rot[0] as f32, rot[1] as f32, rot[2] as f32),
        scale,
    }
}

fn joint_control_ui(
    mut contexts: EguiContexts,
    mut query: Query<(&mut JointComponent, &mut Transform)>,
) {
    use bevy_egui::egui::{self, Slider};

    egui::Window::new("Joint Control").show(contexts.ctx_mut(), |ui| {
        for (mut joint_component, mut transform) in query.iter_mut() {
            ui.label(&joint_component.name);

            let mut position = joint_component.current_position;

            if joint_component.joint_type == urdf_rs::JointType::Revolute
                || joint_component.joint_type == urdf_rs::JointType::Continuous
            {
                let response = ui.add(
                    Slider::new(&mut position, -std::f32::consts::PI..=std::f32::consts::PI)
                        .text("Angle"),
                );
                if response.changed() {
                    joint_component.current_position = position;

                    let axis = Vec3::new(
                        joint_component.axis[0] as f32,
                        joint_component.axis[1] as f32,
                        joint_component.axis[2] as f32,
                    );

                    let rotation = Quat::from_axis_angle(axis.normalize(), position);

                    transform.rotation = rotation;
                }
            } else if joint_component.joint_type == urdf_rs::JointType::Prismatic {
                let response = ui.add(Slider::new(&mut position, -1.0..=1.0).text("Position"));
                if response.changed() {
                    joint_component.current_position = position;

                    let axis = Vec3::new(
                        joint_component.axis[0] as f32,
                        joint_component.axis[1] as f32,
                        joint_component.axis[2] as f32,
                    );

                    let translation = axis.normalize() * position;

                    transform.translation = translation;
                }
            } else {
                ui.label(format!(
                    "Unsupported joint type: {:?}",
                    joint_component.joint_type
                ));
            }
        }
    });
}
