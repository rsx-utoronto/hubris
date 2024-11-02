use bevy::prelude::*;
use bevy_flycam::prelude::*;
use bevy_third_person_camera::*;

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
            transform: Transform::from_xyz(-1.0, 1.0, 2.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        },
        ThirdPersonCamera {
            zoom: Zoom::new(0.1, 50.0),
            cursor_lock_key: KeyCode::Escape,
            ..Default::default()
        },
        FlyCam,
    );

    commands.spawn(camera);
}
