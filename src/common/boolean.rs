//! Solid operators adapt the shared expression to the native backend.
use crate::{common::error::Error, traits::SolidStruct};
use boolean_expression::{Expression, Operation};
use std::ops::{Add, Mul, Sub};

pub struct Boolean<S: SolidStruct> {
	pub(crate) expression: Expression<S>,
}

impl<S: SolidStruct> Boolean<S> {
	pub fn build(self) -> Result<S, Error> {
		let pieces = self.build_vec()?;
		let [solid]: [S; 1] = pieces.try_into().map_err(|pieces: Vec<S>| Error::NotOne(pieces.len()))?;
		Ok(solid)
	}

	pub fn build_vec(self) -> Result<Vec<S>, Error> {
		S::boolean_build(&self)
	}
}

impl<S: SolidStruct> Default for Boolean<S> {
	fn default() -> Self {
		Self { expression: Expression::default() }
	}
}

impl<S: SolidStruct> Clone for Boolean<S> {
	fn clone(&self) -> Self {
		Self { expression: self.expression.map(S::boolean_operand) }
	}
}

impl<S: SolidStruct> From<S> for Boolean<S> {
	fn from(solid: S) -> Self {
		Self { expression: Expression::from(solid) }
	}
}

impl<S: SolidStruct> From<&S> for Boolean<S> {
	fn from(solid: &S) -> Self {
		Self::from(solid.boolean_operand())
	}
}

impl<S: SolidStruct> TryFrom<Boolean<S>> for Vec<S> {
	type Error = Error;
	fn try_from(expression: Boolean<S>) -> Result<Self, Error> {
		expression.build_vec()
	}
}

macro_rules! operator {
	($trait:ident, $method:ident, $operation:ident) => {
		impl<S: SolidStruct, R: Into<Self>> $trait<R> for Boolean<S> {
			type Output = Self;
			fn $method(self, right: R) -> Self {
				Self { expression: self.expression.combine(Operation::$operation, right.into().expression) }
			}
		}
	};
}
operator!(Add, add, Union);
operator!(Sub, sub, Difference);
operator!(Mul, mul, Intersection);
