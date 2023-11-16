use std::{collections::VecDeque, f64::consts::PI, sync::Arc};

use crossbeam::atomic::AtomicCell;
use fxhash::FxHashMap;
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};
use serde::Deserialize;

#[derive(Deserialize, Default)]
struct RigidBodyToml {
    x: f64,
    y: f64,
    z: f64,
    yaw: f64,
    pitch: f64,
    roll: f64,
    moveable: bool,

    children: FxHashMap<String, Self>,
}

pub struct RigidBody {
    translation: Point3<f64>,
    orientation: UnitQuaternion<f64>,
    moveable: bool,
    children: FxHashMap<String, Self>,
}

impl RigidBody {
    fn from_toml_obj(obj: RigidBodyToml) -> Self {
        Self {
            translation: Point3::new(obj.x, obj.y, obj.z).into(),
            orientation: (UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                obj.yaw / 180.0 * PI,
            ) * UnitQuaternion::from_axis_angle(
                &Vector3::x_axis(),
                obj.pitch / 180.0 * PI,
            ) * UnitQuaternion::from_axis_angle(
                &Vector3::z_axis(),
                obj.roll / 180.0 * PI,
            ))
            .into(),
            children: obj
                .children
                .into_iter()
                .map(|(name, obj)| (name.into_boxed_str().into(), Self::from_toml_obj(obj)))
                .collect(),
            moveable: obj.moveable,
        }
    }

    pub fn from_toml(data: &str) -> anyhow::Result<Self> {
        let out = Self::from_toml_obj(toml::from_str(data)?);
        if out.translation != Default::default() {
            Err(anyhow::anyhow!("Base RigidBody has a non-zero translation"))
        } else if out.orientation != Default::default() {
            Err(anyhow::anyhow!(
                "Base RigidBody has a non-default orientation"
            ))
        } else {
            Ok(out)
        }
    }

    pub fn destructure(self) -> impl Iterator<Item = RigidBodyRef> {
        let mut out = vec![];
        let mut process_queue = VecDeque::with_capacity(self.children.len());

        for (name, child) in self.children {
            let transform = Isometry3::from_parts(self.translation.into(), self.orientation);
            let transform = if self.moveable {
                Transform::Dynamic(Arc::new(AtomicCell::new(transform)))
            } else {
                Transform::Static(transform)
            };

            process_queue.push_back((name, child, vec![transform]));
        }

        while let Some((name, child, transforms)) = process_queue.pop_front() {
            for (subname, subchild) in child.children {
                let mut subtransforms = transforms.clone();
                let transform =
                    Isometry3::from_parts(subchild.translation.into(), subchild.orientation);

                if subchild.moveable {
                    subtransforms.push(Transform::Dynamic(Arc::new(AtomicCell::new(transform))));
                } else if let Transform::Static(last_transform) = subtransforms.last().unwrap() {
                    *subtransforms.last_mut().unwrap() =
                        Transform::Static(last_transform * transform);
                } else {
                    subtransforms.push(Transform::Static(transform));
                }

                process_queue.push_back((name.clone() + "/" + &subname, subchild, subtransforms));
            }

            if child.moveable {
                out.push(RigidBodyRef::Dynamic(DynamicRigidBodyRef {
                    name: name.into_boxed_str().into(),
                    transforms: transforms.into_boxed_slice().into(),
                }));
            } else {
                out.push(RigidBodyRef::Static(StaticRigidBodyRef {
                    name: name.into_boxed_str().into(),
                    transforms: transforms.into_boxed_slice().into(),
                }));
            }
        }

        out.into_iter()
    }
}

#[derive(Clone)]
enum Transform {
    Dynamic(Arc<AtomicCell<Isometry3<f64>>>),
    Static(Isometry3<f64>),
}

pub enum RigidBodyRef {
    Dynamic(DynamicRigidBodyRef),
    Static(StaticRigidBodyRef),
}

impl RigidBodyRef {
    /// Gets the name (which includes the names of all the parents) as a `str`
    pub fn get_name(&self) -> &str {
        match self {
            RigidBodyRef::Dynamic(x) => &x.name,
            RigidBodyRef::Static(x) => &x.name,
        }
    }

    /// Internally, the name is stored in an `Arc`. If you need an `Arc<str>` of
    /// the name for whatever reason, you can get a reference to it with this
    /// method.
    pub fn get_arc_name(&self) -> &Arc<str> {
        match self {
            RigidBodyRef::Dynamic(x) => &x.name,
            RigidBodyRef::Static(x) => &x.name,
        }
    }

    pub fn get_global_isometry(&self) -> Isometry3<f64> {
        match self {
            RigidBodyRef::Dynamic(x) => x.get_global_isometry(),
            RigidBodyRef::Static(x) => x.get_global_isometry(),
        }
    }

    pub fn get_local_isometry(&self) -> Isometry3<f64> {
        match self {
            RigidBodyRef::Dynamic(x) => x.get_local_isometry(),
            RigidBodyRef::Static(x) => x.get_local_isometry(),
        }
    }

    pub fn get_global_isometry_f32(&self) -> Isometry3<f32> {
        match self {
            RigidBodyRef::Dynamic(x) => x.get_global_isometry_f32(),
            RigidBodyRef::Static(x) => x.get_global_isometry_f32(),
        }
    }

    pub fn get_local_isometry_f32(&self) -> Isometry3<f32> {
        match self {
            RigidBodyRef::Dynamic(x) => x.get_local_isometry_f32(),
            RigidBodyRef::Static(x) => x.get_local_isometry_f32(),
        }
    }
}

#[derive(Clone)]
pub struct StaticRigidBodyRef {
    name: Arc<str>,
    transforms: Arc<[Transform]>,
}

impl StaticRigidBodyRef {
    pub fn identity(name: impl Into<String>) -> Self {
        Self {
            name: name.into().into_boxed_str().into(),
            transforms: [Transform::Static(Isometry3::default())].into(),
        }
    }

    /// Gets the name (which includes the names of all the parents) as a `str`
    pub fn get_name(&self) -> &str {
        &self.name
    }

    /// Internally, the name is stored in an `Arc`. If you need an `Arc<str>` of
    /// the name for whatever reason, you can get a reference to it with this
    /// method.
    pub fn get_arc_name(&self) -> &Arc<str> {
        &self.name
    }

    pub fn get_global_isometry(&self) -> Isometry3<f64> {
        let mut out = Isometry3::default();

        for transform in self.transforms.iter() {
            match transform {
                Transform::Dynamic(x) => out = x.load() * out,
                Transform::Static(x) => out = x * out,
            }
        }

        out
    }

    pub fn get_global_isometry_f32(&self) -> Isometry3<f32> {
        downcast_isometry(self.get_global_isometry())
    }

    pub fn get_local_isometry(&self) -> Isometry3<f64> {
        let Transform::Static(x) = self.transforms.last().unwrap() else {
            unreachable!();
        };
        *x
    }

    pub fn get_local_isometry_f32(&self) -> Isometry3<f32> {
        downcast_isometry(self.get_local_isometry())
    }
}

#[derive(Clone)]
pub struct DynamicRigidBodyRef {
    name: Arc<str>,
    transforms: Arc<[Transform]>,
}

impl DynamicRigidBodyRef {
    pub fn identity(name: impl Into<String>) -> Self {
        Self {
            name: name.into().into_boxed_str().into(),
            transforms: [Transform::Static(Isometry3::default())].into(),
        }
    }

    /// Gets the name (which includes the names of all the parents) as a `str`
    pub fn get_name(&self) -> &str {
        &self.name
    }

    /// Internally, the name is stored in an `Arc`. If you need an `Arc<str>` of
    /// the name for whatever reason, you can get a reference to it with this
    /// method.
    pub fn get_arc_name(&self) -> &Arc<str> {
        &self.name
    }

    pub fn get_global_isometry(&self) -> Isometry3<f64> {
        let mut out = Isometry3::default();

        for transform in self.transforms.iter() {
            match transform {
                Transform::Dynamic(x) => out = x.load() * out,
                Transform::Static(x) => out = x * out,
            }
        }

        out
    }

    pub fn get_global_isometry_f32(&self) -> Isometry3<f32> {
        downcast_isometry(self.get_global_isometry())
    }

    pub fn get_local_isometry(&self) -> Isometry3<f64> {
        let Transform::Dynamic(x) = self.transforms.last().unwrap() else {
            unreachable!();
        };
        x.load()
    }

    pub fn get_local_isometry_f32(&self) -> Isometry3<f32> {
        downcast_isometry(self.get_local_isometry())
    }

    pub fn set_local_isometry(&self, value: Isometry3<f64>) {
        let Transform::Dynamic(x) = self.transforms.last().unwrap() else {
            unreachable!();
        };
        x.store(value);
    }
}

fn downcast_isometry(high: Isometry3<f64>) -> Isometry3<f32> {
    Isometry3::from_parts(
        Translation3::new(
            high.translation.x as f32,
            high.translation.y as f32,
            high.translation.z as f32,
        ),
        UnitQuaternion::new_normalize(Quaternion::new(
            high.rotation.w as f32,
            high.rotation.i as f32,
            high.rotation.j as f32,
            high.rotation.k as f32,
        )),
    )
}
