#![forbid(unsafe_code)]
#![warn(missing_docs)]
//! Damped motion translated from Daniel Holden’s MIT-licensed animation code.
//! All times are seconds. Nonpositive update deltas freeze dynamic state.
//! Original discussion: <https://theorangeduck.com/page/spring-roll-call>.
/// Convert functions.
pub mod convert;
pub use convert::*;
/// Damper functions.
pub mod damper;
pub use damper::*;
/// Spring functions.
pub mod spring;
pub use spring::*;
/// Composite functions.
pub mod composite;
pub use composite::*;
/// Tracking functions.
pub mod tracking;
pub use tracking::*;
/// Predict functions.
pub mod predict;
pub use predict::*;
/// Inertialize functions.
pub mod inertialize;
pub use inertialize::*;
/// Dead blend functions.
pub mod dead_blend;
pub use dead_blend::*;
/// Easing functions.
pub mod easing;
pub use easing::*;
#[cfg(feature = "glam")]
/// Value spring forms.
pub mod value;
#[cfg(feature = "glam")]
pub use value::*;
#[cfg(feature = "glam")]
/// Quat spring forms.
pub mod quat;
#[cfg(feature = "glam")]
pub use quat::*;
#[cfg(feature = "bevy")]
/// Bevy transform and generic-value spring integration.
pub mod bevy_layer;
#[cfg(feature = "bevy")]
pub use bevy_layer::*;
