#[macro_use]
extern crate approx;

use std::f32::consts::PI;

use glam::{DQuat, DVec3, Quat};
use glam::{Vec2, Vec3};

#[test]
fn test_identity_quaternion() {
    // Identity quaternion, no rotation
    assert_eq!(Quat::IDENTITY, Quat::from_xyzw(0.0, 0.0, 0.0, 1.0));
}

#[test]
fn test_zero_rotations() {
    // Zero rotations should be identity rotations
    assert_eq!(Quat::from_rotation_x(0.).is_near_identity(), true);
    assert_eq!(Quat::from_rotation_y(0.).is_near_identity(), true);
    assert_eq!(Quat::from_rotation_z(0.).is_near_identity(), true);
}

#[test]
fn test() {
    // PI rotation and identity rotation angle should be PI
    assert_relative_eq!(
        Quat::from_rotation_x(PI).angle_between(Quat::IDENTITY),
        PI,
        epsilon = 1e-6
    );

    // Two half-PI's rotations to opposite directions equal to full PI
    assert_relative_eq!(
        Quat::from_rotation_x(PI * 0.5).angle_between(Quat::from_rotation_x(-PI * 0.5)),
        PI,
        epsilon = 1e-6
    );

    // assert_eq!(Quat::IDENTITY.dot(Quat::from_rotation_x(PI * 0.5)), PI);

    // Interpolate with `0.5`
    assert_relative_eq!(
        Quat::IDENTITY.lerp(Quat::from_rotation_x(PI * 0.5), 0.5),
        Quat::from_rotation_x(PI * 0.25)
    );

    // Scaled axis rotation should be zero for identity rotation
    assert_eq!(Quat::IDENTITY.to_scaled_axis(), Vec3::ZERO);

    // Scaled axis X should be equal to initial rotation
    assert_relative_eq!(
        Quat::from_rotation_x(0.5 * PI).to_scaled_axis(),
        Vec3::new(0.5 * PI, 0., 0.),
    );

    assert_eq!(
        Quat::IDENTITY.to_euler(glam::EulerRot::default()),
        (0., 0., 0.)
    );

    //assert_relative_eq!(
    //    Quat::from_rotation_x(0.5 * PI).to_euler(glam::EulerRot::default()),
    //    (0., 0.5 * PI, 0.)
    //);

    // Identity rotation axis angle should be around axis x
    assert_eq!(Quat::IDENTITY.to_axis_angle(), (Vec3::X, 0.0));

    // Rotate 0.5PI around axis X --> axis angle = X, angle in radians 0.5PI
    assert_relative_eq!(Quat::from_rotation_x(0.5 * PI).to_axis_angle().0, Vec3::X);
    assert_relative_eq!(Quat::from_rotation_x(0.5 * PI).to_axis_angle().1, 0.5 * PI);

    // Same but around Y-axis
    assert_relative_eq!(Quat::from_rotation_y(0.5 * PI).to_axis_angle().0, Vec3::Y);
    assert_relative_eq!(Quat::from_rotation_y(0.5 * PI).to_axis_angle().1, 0.5 * PI);

    // If multiplying rotations around axis Y, axis angle remains Y
    assert_relative_eq!(
        (Quat::from_rotation_y(0.5 * PI) * Quat::from_rotation_y(0.5 * PI))
            .to_axis_angle()
            .0,
        Vec3::Y
    );

    // Angles are being added together
    assert_relative_eq!(
        (Quat::from_rotation_y(0.5 * PI) * Quat::from_rotation_y(0.5 * PI))
            .to_axis_angle()
            .1,
        PI
    );

    // TODO why?
    assert_relative_eq!(
        (Quat::from_rotation_x(0.5 * PI) * Quat::from_rotation_y(0.5 * PI))
            .to_axis_angle()
            .0,
        Vec3::new(0.5773502, 0.5773502, 0.5773502)
    );

    // TODO why?
    assert_relative_eq!(
        (Quat::from_rotation_x(0.5 * PI) * Quat::from_rotation_y(0.5 * PI))
            .to_axis_angle()
            .1,
        2.0943952
    );

    assert_relative_eq!(
        Quat::from_axis_angle(Vec3::X, 0.5 * PI).mul_vec3(Vec3::X),
        Vec3::X
    );

    assert_relative_eq!(
        Quat::from_axis_angle(Vec3::X, 0.5 * PI).mul_vec3(Vec3::Y),
        Vec3::Z
    );

    assert_relative_eq!(
        Quat::from_axis_angle(Vec3::Y, PI).mul_vec3(Vec3::X),
        -Vec3::X
    );

    assert_relative_eq!(
        Quat::from_axis_angle(Vec3::Z, 0.5 * PI).mul_vec3(Vec3::Y),
        -Vec3::X
    );

    assert_relative_eq!(
        DQuat::from_axis_angle(DVec3::Z, 2. * std::f64::consts::PI).mul_vec3(DVec3::Y),
        DVec3::Y,
        epsilon = 1e-6
    );

    assert_relative_eq!(
        DQuat::from_axis_angle(DVec3::Z, 2. * std::f64::consts::PI).mul_vec3(DVec3::Z),
        DVec3::Z,
        epsilon = 1e-6
    );

    assert_eq!(
        Quat::from_rotation_arc(Vec3::X, Vec3::Y),
        Quat::from_rotation_z(0.5 * PI)
    );

    assert_eq!(
        Quat::from_scaled_axis(Vec3::new(0., 0.5 * PI, 0.)),
        Quat::from_rotation_y(0.5 * PI)
    );

    assert_eq!(
        Quat::from_rotation_x(0.5 * PI).inverse(),
        Quat::from_rotation_x(-0.5 * PI)
    );
}

#[test]
fn test_vec3() {
    assert_eq!(Vec3::X, Vec3::new(1., 0., 0.));
    assert_relative_eq!(Vec3::X.angle_between(Vec3::Y), PI / 2.0);
    assert_relative_eq!(Vec3::X.angle_between(-Vec3::X), PI);
    assert_eq!(Vec3::X.any_orthogonal_vector(), Vec3::Z);
    assert_eq!(Vec3::Y.any_orthogonal_vector(), -Vec3::Z);
    assert_eq!(Vec3::Z.any_orthogonal_vector(), Vec3::Y);
    assert_eq!(Vec3::new(-5., -5., -5.).abs(), Vec3::new(5., 5., 5.));
    assert_eq!(Vec3::X.cross(Vec3::Y), Vec3::Z);
    assert_eq!(Vec3::X.cross(-Vec3::Y), -Vec3::Z);
    assert_eq!(Vec3::X.cross(-Vec3::X), Vec3::ZERO);
    assert_eq!(-Vec3::X.cross(Vec3::Y), -Vec3::Z);
    assert_eq!(Vec3::Z.cross(-Vec3::X), -Vec3::Y);
    assert_eq!(Vec3::X.distance(Vec3::new(1., 2., 0.)), 2.);
    assert_eq!(Vec3::X.distance(Vec3::new(-5., 0., 0.)), 6.);
    assert_relative_eq!(Vec3::X.distance_squared(Vec3::new(-5., 0., 0.)), 36.);
    assert_eq!(Vec3::X.dot(Vec3::X), 1.);
    assert_eq!(Vec3::X.dot(-Vec3::X), -1.);
    assert_eq!(Vec3::X.dot(Vec3::Y), 0.);
    assert_eq!(Vec3::X.dot(Vec3::Z), 0.);
    assert_eq!(Vec3::X.dot(-Vec3::Z), 0.);
    assert_eq!(Vec3::new(5., 0., 0.).dot(Vec3::new(5., 0., 0.)), 25.); // why?
    assert_eq!(
        Vec3::new(5., 0., 0.)
            .normalize()
            .dot(Vec3::new(5., 0., 0.).normalize()),
        1.
    );
    assert_eq!(Vec3::X.lerp(Vec3::ZERO, 1.), Vec3::ZERO);
    assert_eq!(Vec3::X.lerp(Vec3::ZERO, 0.5), Vec3::new(0.5, 0., 0.));
    assert_eq!(Vec3::X.lerp(Vec3::ZERO, 0.25), Vec3::new(0.75, 0., 0.));
    assert_eq!(
        Vec3::new(5., 5., 0.).normalize(),
        Vec3::new(0.7071068, 0.7071068, 0.0)
    );
    assert_eq!(
        Vec3::new(2., 2., 2.).normalize(),
        Vec3::new(0.57735026, 0.57735026, 0.57735026)
    );
    assert_eq!(Vec3::new(1.5, 0.5, 2.2).round(), Vec3::new(2., 1., 2.));
    assert_eq!(Vec3::X.to_array(), [1., 0., 0.]);
    assert_eq!(Vec3::X.to_string(), "[1, 0, 0]".to_string());
    assert_eq!(Vec3::X.truncate(), Vec2::new(1., 0.));
    assert_eq!(Vec3::X.try_normalize(), Some(Vec3::X));
    assert_eq!(Vec3::ZERO.try_normalize(), None);
}
