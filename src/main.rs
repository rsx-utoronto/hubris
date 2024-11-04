use avian3d::prelude::*;
use bevy::prelude::*;
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
        ))
        .insert_gizmo_config(
            PhysicsGizmos {
                axis_lengths: None,
                ..default()
            },
            GizmoConfig::default(),
        )
        .insert_resource(SubstepCount(200))
        .add_systems(Startup, spawn_robots)
        .add_systems(Update, (apply_joint_torque_system, collision_callback))
        .run();
}

fn spawn_robots(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_robot(
        Transform {
            translation: Vec3::new(0.0, 0.0, 0.0),
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            scale: Vec3::ONE,
        },
        &mut commands,
        &mut meshes,
        &asset_server,
        &mut materials,
    );
    spawn_robot(
        Transform {
            translation: Vec3::new(0.0, 10.0, 0.0),
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            scale: Vec3::ONE,
        },
        &mut commands,
        &mut meshes,
        &asset_server,
        &mut materials,
    );
}

fn spawn_robot(
    base_transform: Transform,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    asset_server: &Res<AssetServer>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let urdf_path = "sample_description/urdf/low_cost_robot.urdf";
    let robot = urdf_rs::read_file(urdf_path).expect("Failed to read URDF file");

    let mut link_entities = HashMap::new();
    let mut link_transforms = HashMap::new();

    for link in &robot.links {
        let initial_transform = if link.name == "base_link" {
            base_transform
        } else {
            Transform::IDENTITY
        };

        let mass_properties = link_to_mass_properties(link);

        let link_entity = match link.name.as_str() {
            "base_link" => commands.spawn((
                initial_transform,
                RigidBody::Kinematic,
                Restitution::ZERO,
                mass_properties,
                InheritedVisibility::VISIBLE,
                GlobalTransform::IDENTITY,
            )),
            _ => commands.spawn((
                initial_transform,
                RigidBody::Dynamic,
                Restitution::ZERO,
                mass_properties,
                InheritedVisibility::VISIBLE,
                GlobalTransform::IDENTITY,
            )),
        };
        let link_entity_id = link_entity.id();
        link_entities.insert(link.name.clone(), link_entity_id);
        link_transforms.insert(link.name.clone(), initial_transform);

        for visual in &link.visual {
            let (mesh_handle, material_handle) =
                visual_to_mesh_and_material(visual, meshes, asset_server, materials);
            let transform = urdf_to_transform(&visual.origin, &Some(visual.geometry.clone()));

            commands.entity(link_entity_id).with_children(|parent| {
                parent.spawn((PbrBundle {
                    mesh: mesh_handle,
                    material: material_handle,
                    transform,
                    ..Default::default()
                },));
            });
        }

        for collision in &link.collision {
            let mesh_handle = collision_to_mesh(collision, meshes, asset_server);
            let transform = urdf_to_transform(&collision.origin, &Some(collision.geometry.clone()));

            commands.entity(link_entity_id).with_children(|parent| {
                parent.spawn((
                    mesh_handle,
                    transform,
                    ColliderConstructor::ConvexDecompositionFromMesh,
                ));
            });
        }
    }

    for joint in &robot.joints {
        let parent_link_name = &joint.parent.link;
        let child_link_name = &joint.child.link;
        let joint_transform = urdf_to_transform(&joint.origin, &None);

        if let (Some(parent_entity), Some(child_entity)) = (
            link_entities.get(parent_link_name),
            link_entities.get(child_link_name),
        ) {
            if let Some(parent_transform) = link_transforms.get(parent_link_name) {
                let accumulated_transform = *parent_transform * joint_transform;
                link_transforms.insert(child_link_name.clone(), accumulated_transform);

                commands.entity(*child_entity).insert(accumulated_transform);
            }

            urdf_to_joint(commands, *parent_entity, *child_entity, joint);
        }
    }
}

fn link_to_mass_properties(link: &urdf_rs::Link) -> MassPropertiesBundle {
    MassPropertiesBundle {
        mass: Mass(link.inertial.mass.value as f32),
        inertia: Inertia(avian3d::math::Matrix3 {
            x_axis: Vec3::new(
                link.inertial.inertia.ixx as f32,
                link.inertial.inertia.ixy as f32,
                link.inertial.inertia.ixz as f32,
            ),
            y_axis: Vec3::new(
                link.inertial.inertia.ixy as f32,
                link.inertial.inertia.iyy as f32,
                link.inertial.inertia.iyz as f32,
            ),
            z_axis: Vec3::new(
                link.inertial.inertia.ixz as f32,
                link.inertial.inertia.iyz as f32,
                link.inertial.inertia.izz as f32,
            ),
        }),
        center_of_mass: CenterOfMass(Vec3::new(
            link.inertial.origin.xyz[0] as f32,
            link.inertial.origin.xyz[1] as f32,
            link.inertial.origin.xyz[2] as f32,
        )),
        ..Default::default()
    }
}

fn visual_to_mesh_and_material(
    visual: &urdf_rs::Visual,
    meshes: &mut ResMut<Assets<Mesh>>,
    asset_server: &Res<AssetServer>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> (Handle<Mesh>, Handle<StandardMaterial>) {
    match &visual.geometry {
        Geometry::Mesh { filename, .. } => {
            let mesh_handle = asset_server.load(filename);
            let material_handle = create_material(&visual.material, materials);
            (mesh_handle, material_handle)
        }
        Geometry::Box { size } => {
            let mesh = Mesh::from(Cuboid::new(size[0] as f32, size[1] as f32, size[2] as f32));
            let mesh_handle = meshes.add(mesh);
            let material_handle = create_material(&visual.material, materials);
            (mesh_handle, material_handle)
        }
        Geometry::Cylinder { radius, length } => {
            let mesh = Mesh::from(Cylinder {
                radius: *radius as f32,
                half_height: (*length as f32) / 2.0,
            });
            let mesh_handle = meshes.add(mesh);
            let material_handle = create_material(&visual.material, materials);
            (mesh_handle, material_handle)
        }
        Geometry::Sphere { radius } => {
            let mesh = Mesh::from(Sphere {
                radius: *radius as f32,
            });
            let mesh_handle = meshes.add(mesh);
            let material_handle = create_material(&visual.material, materials);
            (mesh_handle, material_handle)
        }
        _ => {
            warn!("Unsupported geometry type: {:?}", visual.geometry);
            (Handle::default(), Handle::default())
        }
    }
}

fn collision_to_mesh(
    collision: &urdf_rs::Collision,
    meshes: &mut ResMut<Assets<Mesh>>,
    asset_server: &Res<AssetServer>,
) -> Handle<Mesh> {
    match &collision.geometry {
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
            warn!("Unsupported geometry type: {:?}", collision.geometry);
            Handle::default()
        }
    }
}

fn create_material(
    urdf_material: &Option<urdf_rs::Material>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> Handle<StandardMaterial> {
    let color = if let Some(material) = urdf_material {
        if let Some(urdf_texture) = &material.texture {
            warn!("Textures are not supported yet: {:?}", urdf_texture);
            Color::srgba(0.8, 0.8, 0.8, 1.0)
        } else if let Some(urdf_color) = &material.color {
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
        metallic: 1.0,
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

fn urdf_to_joint(
    commands: &mut Commands,
    entity1: Entity,
    entity2: Entity,
    joint: &urdf_rs::Joint,
) {
    let dynamics = joint.dynamics.clone().unwrap_or(urdf_rs::Dynamics {
        damping: 10000.0,
        friction: 0.0,
    });

    let axis = Vec3::new(
        joint.axis.xyz[0] as f32,
        joint.axis.xyz[1] as f32,
        joint.axis.xyz[2] as f32,
    );

    let anchor = Vec3::new(
        joint.origin.xyz[0] as f32,
        joint.origin.xyz[1] as f32,
        joint.origin.xyz[2] as f32,
    );

    match joint.joint_type {
        urdf_rs::JointType::Revolute => {
            let joint = RevoluteJoint::new(entity1, entity2)
                .with_aligned_axis(axis)
                .with_angular_velocity_damping(dynamics.damping as f32)
                .with_compliance(0.0)
                .with_angle_limits(joint.limit.lower as f32, joint.limit.upper as f32);

            commands.spawn(joint);
        }
        urdf_rs::JointType::Continuous => {
            let joint = RevoluteJoint::new(entity1, entity2)
                .with_aligned_axis(axis)
                .with_local_anchor_1(anchor)
                .with_angular_velocity_damping(dynamics.damping as f32)
                .with_compliance(0.0);

            commands.spawn(joint);
        }
        urdf_rs::JointType::Fixed => {
            let joint = FixedJoint::new(entity1, entity2);

            commands.spawn(joint);
        }
        _ => {
            error!("Unsupported joint type: {:?}", joint.joint_type);
        }
    }
}

fn apply_joint_torque_system(mut joint_query: Query<&mut RevoluteJoint>) {
    for joint in joint_query.iter_mut() {
        let _child = joint.entity1;
        let _parent = joint.entity2;
    }
}

fn collision_callback(mut collisions: ResMut<Collisions>) {
    collisions.retain(ignore_collision(0.01));
}

fn ignore_collision(ignore_threshold: f32) -> impl Fn(&mut Contacts) -> bool {
    move |contacts: &mut Contacts| {
        contacts.manifolds.iter().all(|manifold| {
            manifold
                .contacts
                .iter()
                .all(|contact| contact.penetration >= ignore_threshold)
        })
    }
}
