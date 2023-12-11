use std::{fmt::Debug, marker::PhantomData};

use burn::{
    module::{AutodiffModule, Module},
    record::Record,
    tensor::{
        activation,
        backend::{AutodiffBackend, Backend},
        Tensor,
    },
};
use serde::{Deserialize, Serialize};

pub mod time_series;

#[derive(Debug, Clone)]
pub struct TwoTuple<A, B>(pub A, pub B);

impl<M1: Record, M2: Record> Record for TwoTuple<M1, M2> {
    type Item<S: burn::record::PrecisionSettings> = (M1::Item<S>, M2::Item<S>);

    fn into_item<S: burn::record::PrecisionSettings>(self) -> Self::Item<S> {
        (self.0.into_item(), self.1.into_item())
    }

    fn from_item<S: burn::record::PrecisionSettings>((m1, m2): Self::Item<S>) -> Self {
        Self(Record::from_item(m1), Record::from_item(m2))
    }
}

impl<M1: Module<B>, M2: Module<B>, B: Backend> Module<B> for TwoTuple<M1, M2> {
    type Record = TwoTuple<M1::Record, M2::Record>;

    fn collect_devices(&self, mut devices: burn::module::Devices<B>) -> burn::module::Devices<B> {
        devices = self.0.collect_devices(devices);
        devices = self.1.collect_devices(devices);
        devices
    }

    fn fork(mut self, device: &<B as Backend>::Device) -> Self {
        self.0 = self.0.fork(device);
        self.1 = self.1.fork(device);
        self
    }

    fn to_device(mut self, device: &<B as Backend>::Device) -> Self {
        self.0 = self.0.to_device(device);
        self.1 = self.1.to_device(device);
        self
    }

    fn visit<V: burn::module::ModuleVisitor<B>>(&self, visitor: &mut V) {
        self.0.visit(visitor);
        self.1.visit(visitor);
    }

    fn map<M: burn::module::ModuleMapper<B>>(mut self, mapper: &mut M) -> Self {
        self.0 = self.0.map(mapper);
        self.1 = self.1.map(mapper);
        self
    }

    fn load_record(mut self, record: Self::Record) -> Self {
        self.0 = self.0.load_record(record.0);
        self.1 = self.1.load_record(record.1);
        self
    }

    fn into_record(self) -> Self::Record {
        TwoTuple(self.0.into_record(), self.1.into_record())
    }
}

impl<M1: AutodiffModule<B>, M2: AutodiffModule<B>, B: AutodiffBackend> AutodiffModule<B>
    for TwoTuple<M1, M2>
{
    type InnerModule = TwoTuple<M1::InnerModule, M2::InnerModule>;

    fn valid(&self) -> Self::InnerModule {
        TwoTuple(self.0.valid(), self.1.valid())
    }
}

#[derive(Debug, Clone)]
pub struct ThreeTuple<A, B, C>(pub A, pub B, pub C);

impl<M1: Record, M2: Record, M3: Record> Record for ThreeTuple<M1, M2, M3> {
    type Item<S: burn::record::PrecisionSettings> = (M1::Item<S>, M2::Item<S>, M3::Item<S>);

    fn into_item<S: burn::record::PrecisionSettings>(self) -> Self::Item<S> {
        (self.0.into_item(), self.1.into_item(), self.2.into_item())
    }

    fn from_item<S: burn::record::PrecisionSettings>((m1, m2, m3): Self::Item<S>) -> Self {
        Self(
            Record::from_item(m1),
            Record::from_item(m2),
            Record::from_item(m3),
        )
    }
}

impl<M1: Module<B>, M2: Module<B>, M3: Module<B>, B: Backend> Module<B> for ThreeTuple<M1, M2, M3> {
    type Record = ThreeTuple<M1::Record, M2::Record, M3::Record>;

    fn collect_devices(&self, mut devices: burn::module::Devices<B>) -> burn::module::Devices<B> {
        devices = self.0.collect_devices(devices);
        devices = self.1.collect_devices(devices);
        devices = self.2.collect_devices(devices);
        devices
    }

    fn fork(mut self, device: &<B as Backend>::Device) -> Self {
        self.0 = self.0.fork(device);
        self.1 = self.1.fork(device);
        self.2 = self.2.fork(device);
        self
    }

    fn to_device(mut self, device: &<B as Backend>::Device) -> Self {
        self.0 = self.0.to_device(device);
        self.1 = self.1.to_device(device);
        self.2 = self.2.to_device(device);
        self
    }

    fn visit<V: burn::module::ModuleVisitor<B>>(&self, visitor: &mut V) {
        self.0.visit(visitor);
        self.1.visit(visitor);
        self.2.visit(visitor);
    }

    fn map<M: burn::module::ModuleMapper<B>>(mut self, mapper: &mut M) -> Self {
        self.0 = self.0.map(mapper);
        self.1 = self.1.map(mapper);
        self.2 = self.2.map(mapper);
        self
    }

    fn load_record(mut self, record: Self::Record) -> Self {
        self.0 = self.0.load_record(record.0);
        self.1 = self.1.load_record(record.1);
        self.2 = self.2.load_record(record.2);
        self
    }

    fn into_record(self) -> Self::Record {
        ThreeTuple(
            self.0.into_record(),
            self.1.into_record(),
            self.2.into_record(),
        )
    }
}

impl<M1: AutodiffModule<B>, M2: AutodiffModule<B>, M3: AutodiffModule<B>, B: AutodiffBackend>
    AutodiffModule<B> for ThreeTuple<M1, M2, M3>
{
    type InnerModule = ThreeTuple<M1::InnerModule, M2::InnerModule, M3::InnerModule>;

    fn valid(&self) -> Self::InnerModule {
        ThreeTuple(self.0.valid(), self.1.valid(), self.2.valid())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Activation {
    Relu,
    Gelu,
    Sigmoid,
    Tanh,
    None,
}

impl Activation {
    pub fn forward<const D: usize, B: Backend>(self, tensor: Tensor<B, D>) -> Tensor<B, D> {
        match self {
            Activation::Relu => activation::relu(tensor),
            Activation::Gelu => activation::gelu(tensor),
            Activation::Sigmoid => activation::sigmoid(tensor),
            Activation::Tanh => activation::tanh(tensor),
            Activation::None => tensor,
        }
    }
}

impl Record for Activation {
    type Item<S: burn::record::PrecisionSettings> = Self;

    fn into_item<S: burn::record::PrecisionSettings>(self) -> Self::Item<S> {
        self
    }

    fn from_item<S: burn::record::PrecisionSettings>(item: Self::Item<S>) -> Self {
        item
    }
}

impl<B: Backend> Module<B> for Activation {
    type Record = Self;

    fn collect_devices(&self, devices: burn::module::Devices<B>) -> burn::module::Devices<B> {
        devices
    }

    fn fork(self, _device: &<B as Backend>::Device) -> Self {
        self
    }

    fn to_device(self, _device: &<B as Backend>::Device) -> Self {
        self
    }

    fn visit<V: burn::module::ModuleVisitor<B>>(&self, _visitor: &mut V) {}

    fn map<M: burn::module::ModuleMapper<B>>(self, _mapper: &mut M) -> Self {
        self
    }

    fn load_record(self, _record: Self::Record) -> Self {
        self
    }

    fn into_record(self) -> Self::Record {
        self
    }
}

impl<B: AutodiffBackend> AutodiffModule<B> for Activation {
    type InnerModule = Self;

    fn valid(&self) -> Self::InnerModule {
        *self
    }
}

struct PhantomModule<T>(PhantomData<T>);

impl<T> Debug for PhantomModule<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PhantomModule").finish()
    }
}

impl<T> Clone for PhantomModule<T> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<T> Copy for PhantomModule<T> {}
unsafe impl<T> Send for PhantomModule<T> {}
unsafe impl<T> Sync for PhantomModule<T> {}

impl<T, B: Backend> Module<B> for PhantomModule<T> {
    type Record = ();

    fn collect_devices(&self, devices: burn::module::Devices<B>) -> burn::module::Devices<B> {
        devices
    }

    fn fork(self, _device: &<B as Backend>::Device) -> Self {
        self
    }

    fn to_device(self, _device: &<B as Backend>::Device) -> Self {
        self
    }

    fn visit<V: burn::module::ModuleVisitor<B>>(&self, _visitor: &mut V) {}

    fn map<M: burn::module::ModuleMapper<B>>(self, _mapper: &mut M) -> Self {
        self
    }

    fn load_record(self, _record: Self::Record) -> Self {
        self
    }

    fn into_record(self) -> Self::Record {}
}

impl<T, B: AutodiffBackend> AutodiffModule<B> for PhantomModule<T> {
    type InnerModule = Self;

    fn valid(&self) -> Self::InnerModule {
        *self
    }
}

impl<T> Default for PhantomModule<T> {
    fn default() -> Self {
        Self(PhantomData)
    }
}
