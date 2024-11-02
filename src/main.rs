use bevy::prelude::*;
use bevy_fps_counter::FpsCounterPlugin;
use bevy_rapier3d::prelude::*;
use urdf_rs::{Geometry, Robot, Visual};

mod world;
use world::WorldPlugin;

mod camera;
use camera::CameraPlugin;

#[derive(Debug, Resource)]
struct RobotResource {
    robot: Robot,
}

fn main() {
    let urdf_path = "sample_description/urdf/low_cost_robot.urdf";
    let urdf_robot = urdf_rs::read_file(urdf_path).unwrap();

    App::new()
        .add_plugins((
            DefaultPlugins,
            bevy_stl::StlPlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
            FpsCounterPlugin,
            WorldPlugin,
            CameraPlugin,
        ))
        .insert_resource(RobotResource { robot: urdf_robot })
        .add_systems(Startup, spawn_robot)
        .run();
}

fn spawn_robot(
    mut commands: Commands,
    urdf_robot: Res<RobotResource>,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for link in &urdf_robot.robot.links {
        for visual in &link.visual {
            let mut mesh = PbrBundle::default().mesh;

            if let Geometry::Mesh { filename, .. } = &visual.geometry {
                mesh = asset_server.load(filename);
            }

            let mut color = Color::srgba(0.8, 0.8, 0.8, 1.0);

            if let Some(material) = &visual.material {
                if let Some(ref urdf_color) = material.color {
                    color = Color::srgba(
                        urdf_color.rgba[0] as f32,
                        urdf_color.rgba[1] as f32,
                        urdf_color.rgba[2] as f32,
                        urdf_color.rgba[3] as f32,
                    );
                }
            }

            let material_handle = materials.add(StandardMaterial {
                base_color: color,
                ..Default::default()
            });

            let link_pbr = PbrBundle {
                mesh,
                material: material_handle,
                transform: urdf_to_transform(visual),
                ..Default::default()
            };

            commands.spawn(link_pbr);
        }
    }
}

fn urdf_to_transform(visual: &Visual) -> Transform {
    let origin = visual.origin.clone();
    let mut pos = origin.xyz;
    let rot = origin.rpy;

    let mut scale = Vec3::ONE;

    if let Geometry::Mesh {
        scale: Some(mesh_scale),
        ..
    } = &visual.geometry
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
