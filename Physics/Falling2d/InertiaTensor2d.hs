{-# LANGUAGE MultiParamTypeClasses #-}

module Physics.Falling2d.InertiaTensor2d
(
InertiaTensor2d(..),
InverseInertiaTensor2d(..),
)
where

import Data.Vect.Double.Base
import Physics.Falling.Dynamics.InertiaTensor
import Physics.Falling2d.Vec1

newtype InertiaTensor2d        = InertiaTensor2d Double
newtype InverseInertiaTensor2d = InverseInertiaTensor2d Double

instance InertiaTensor InertiaTensor2d InverseInertiaTensor2d Vec1 Proj3 where
  inverseInertia (InertiaTensor2d inertia) = InverseInertiaTensor2d $ if inertia == 0.0 then 0.0 else 1.0 / inertia

instance InverseInertiaTensor InverseInertiaTensor2d Vec1 Proj3 where
  applyToVector (InverseInertiaTensor2d i) v = v &* i
  toWorldSpaceTensor tensor _ _              = tensor

instance Show InertiaTensor2d where
  show (InertiaTensor2d i) = show i

instance Show InverseInertiaTensor2d where
  show (InverseInertiaTensor2d i) = show i
