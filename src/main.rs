use bevy::prelude::*;
use bevy_fps_counter::FpsCounterPlugin;
use bevy_rapier3d::prelude::*;
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
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(
            Startup,
            (
                spawn_robots,
                (process_urdf_visuals, process_urdf_collisions),
            )
                .chain(),
        )
        .run();
}

#[derive(Component)]
struct Robot;

#[derive(Component)]
struct RobotPart;

#[derive(Component)]
struct UrdfVisual {
    geometry: Geometry,
    material: Option<urdf_rs::Material>,
    origin: Pose,
}

#[derive(Component)]
struct URDFCollision {
    geometry: Geometry,
    origin: Pose,
}

fn spawn_robots(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_robot(
        &mut commands,
        &asset_server,
        &mut materials,
        Transform::from_xyz(0.0, 0.3, 0.0),
    );
}

fn spawn_robot(
    commands: &mut Commands,
    _asset_server: &Res<AssetServer>,
    _materials: &mut ResMut<Assets<StandardMaterial>>,
    base_transform: Transform,
) {
    let urdf_path = "sample_description/urdf/low_cost_robot.urdf";
    let robot = urdf_rs::read_file(urdf_path).expect("Failed to read URDF file");

    commands
        .spawn((
            Robot,
            TransformBundle::from_transform(base_transform),
            VisibilityBundle::default(),
        ))
        .with_children(|parent| {
            for link in &robot.links {
                for visual in &link.visual {
                    parent.spawn((
                        RobotPart,
                        UrdfVisual {
                            geometry: visual.geometry.clone(),
                            material: visual.material.clone(),
                            origin: visual.origin.clone(),
                        },
                        TransformBundle::default(),
                        VisibilityBundle::default(),
                    ));
                }

                for collision in &link.collision {
                    parent.spawn((
                        URDFCollision {
                            geometry: collision.geometry.clone(),
                            origin: collision.origin.clone(),
                        },
                        TransformBundle::default(),
                        VisibilityBundle::default(),
                    ));
                }
            }
        });
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
                    half_height: *length as f32,
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
                warn!("Unsupported geometry type");
                continue;
            }
        };

        let transform = urdf_to_transform(&urdf_visual.origin, &urdf_visual.geometry);

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
    for (entity, urdf_visual) in query.iter() {
        let mesh_handle = match &urdf_visual.geometry {
            Geometry::Mesh { filename, .. } => asset_server.load(filename),
            Geometry::Box { size } => {
                let mesh = Mesh::from(Cuboid::new(size[0] as f32, size[1] as f32, size[2] as f32));
                meshes.add(mesh)
            }
            Geometry::Cylinder { radius, length } => {
                let mesh = Mesh::from(Cylinder {
                    radius: *radius as f32,
                    half_height: *length as f32,
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
                warn!("Unsupported geometry type");
                continue;
            }
        };

        let transform = urdf_to_transform(&urdf_visual.origin, &urdf_visual.geometry);

        commands.entity(entity).insert((
            (
                AsyncCollider(ComputedColliderShape::ConvexDecomposition(
                    VHACDParameters::default(),
                )),
                RigidBody::Fixed,
            ),
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

fn urdf_to_transform(origin: &Pose, geometry: &Geometry) -> Transform {
    let mut pos = origin.xyz;
    let rot = origin.rpy;

    let mut scale = Vec3::ONE;

    if let Geometry::Mesh {
        scale: Some(mesh_scale),
        ..
    } = geometry
    {
        scale = Vec3::new(
            mesh_scale[0] as f32,
            mesh_scale[1] as f32,
            mesh_scale[2] as f32,
        );

        pos[0] *= mesh_scale[0];
        pos[1] *= mesh_scale[1];
        pos[2] *= mesh_scale[2];
    }

    Transform {
        translation: Vec3::new(pos[0] as f32, pos[1] as f32, pos[2] as f32),
        rotation: Quat::from_euler(EulerRot::XYZ, rot[0] as f32, rot[1] as f32, rot[2] as f32),
        scale,
    }
}
