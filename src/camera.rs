use bevy::prelude::*;
use bevy_flycam::prelude::*;

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(NoCameraPlayerPlugin)
            .insert_resource(MovementSettings {
                sensitivity: 0.0002,
                speed: 2.0,
            })
            .add_systems(Startup, spawn_camera);
    }
}

fn spawn_camera(mut commands: Commands) {
    let camera = (
        Camera3dBundle {
            transform: Transform::from_xyz(-0.3, 0.3, -0.3)
                .looking_at(Vec3::new(0.0, 0.08, 0.0), Vec3::Y),
            ..Default::default()
        },
        FlyCam,
    );

    commands.spawn(camera);
}
