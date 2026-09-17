//! Boolean algebra without geometry, allocation of intermediate products, or recursion.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Operation {
	Union,
	Difference,
	Intersection,
}

impl Operation {
	pub const fn as_str(self) -> &'static str {
		match self {
			Self::Union => "union",
			Self::Difference => "difference",
			Self::Intersection => "intersection",
		}
	}
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Instruction {
	Operand(usize),
	Combine(Operation),
}

/// A postfix program. Construction preserves its stack and operand invariants.
#[derive(Debug, Clone)]
pub struct Expression<T> {
	operands: Vec<T>,
	program: Vec<Instruction>,
}

impl<T> Default for Expression<T> {
	fn default() -> Self {
		Self { operands: Vec::new(), program: Vec::new() }
	}
}

impl<T> From<T> for Expression<T> {
	fn from(operand: T) -> Self {
		Self { operands: vec![operand], program: vec![Instruction::Operand(0)] }
	}
}

impl<T> Expression<T> {
	pub fn fold(operation: Operation, operands: impl IntoIterator<Item = T>) -> Self {
		let operands: Vec<T> = operands.into_iter().collect();
		let mut program = Vec::with_capacity(operands.len().saturating_mul(2).saturating_sub(1));
		for index in 0..operands.len() {
			program.push(Instruction::Operand(index));
			if index > 0 {
				program.push(Instruction::Combine(operation));
			}
		}
		Self { operands, program }
	}

	pub fn operands(&self) -> &[T] {
		&self.operands
	}
	pub fn instructions(&self) -> &[Instruction] {
		&self.program
	}

	pub fn combine(mut self, operation: Operation, other: Self) -> Self {
		if self.operands.is_empty() {
			return if operation == Operation::Union { other } else { self };
		}
		if other.operands.is_empty() {
			return if operation == Operation::Intersection { other } else { self };
		}
		let offset = self.operands.len();
		self.operands.extend(other.operands);
		self.program.extend(other.program.into_iter().map(|step| match step {
			Instruction::Operand(index) => Instruction::Operand(index + offset),
			step => step,
		}));
		self.program.push(Instruction::Combine(operation));
		self
	}

	pub fn map<'a, U>(&'a self, map: impl FnMut(&'a T) -> U) -> Expression<U> {
		Expression { operands: self.operands.iter().map(map).collect(), program: self.program.clone() }
	}

	/// Empty expressions have no result. Backend values can themselves be fallible.
	pub fn evaluate<R>(&self, mut operand: impl FnMut(&T) -> R, mut combine: impl FnMut(R, Operation, R) -> R) -> Option<R> {
		let mut stack = Vec::new();
		for step in &self.program {
			match *step {
				Instruction::Operand(index) => stack.push(operand(&self.operands[index])),
				Instruction::Combine(operation) => {
					let right = stack.pop().expect("expression has a right operand");
					let left = stack.pop().expect("expression has a left operand");
					stack.push(combine(left, operation, right));
				}
			}
		}
		debug_assert!(stack.len() <= 1);
		stack.pop()
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn grouping_and_difference_are_preserved() {
		let union = Expression::from(0).combine(Operation::Union, Expression::from(1));
		let difference = Expression::fold(Operation::Difference, [2, 3, 4]);
		let expression = union.combine(Operation::Intersection, difference);
		for bits in 0..32 {
			let present = |index: &usize| bits & (1 << index) != 0;
			assert_eq!(
				expression.evaluate(present, |a, op, b| match op {
					Operation::Union => a || b,
					Operation::Difference => a && !b,
					Operation::Intersection => a && b,
				}),
				Some((present(&0) || present(&1)) && present(&2) && !present(&3) && !present(&4))
			);
		}
	}

	#[test]
	fn intersections_of_groups_stay_linear_and_evaluate_without_recursion() {
		let expression = (0..10_000).map(|index| Expression::from(index * 2).combine(Operation::Union, Expression::from(index * 2 + 1))).reduce(|a, b| a.combine(Operation::Intersection, b)).expect("groups");
		assert_eq!(expression.operands().len(), 20_000);
		assert_eq!(expression.instructions().len(), 39_999);
		assert_eq!(
			expression.evaluate(
				|index| index % 2 == 0,
				|a, op, b| match op {
					Operation::Union => a || b,
					Operation::Intersection => a && b,
					Operation::Difference => a && !b,
				}
			),
			Some(true)
		);
	}

	#[test]
	fn empty_set_laws_do_not_call_the_backend() {
		for operation in [Operation::Union, Operation::Difference, Operation::Intersection] {
			assert_eq!(Expression::fold(operation, [7]).evaluate(|value| *value, |_, _, _| unreachable!()), Some(7));
			assert_eq!(Expression::<i32>::fold(operation, []).evaluate(|_| unreachable!(), |_, _, _| unreachable!()), None::<i32>);
			let left = Expression::from(7).combine(operation, Expression::default());
			let right = Expression::default().combine(operation, Expression::from(7));
			assert_eq!(left.evaluate(|value| *value, |_, _, _| unreachable!()), (operation != Operation::Intersection).then_some(7));
			assert_eq!(right.evaluate(|value| *value, |_, _, _| unreachable!()), (operation == Operation::Union).then_some(7));
		}
	}
}
