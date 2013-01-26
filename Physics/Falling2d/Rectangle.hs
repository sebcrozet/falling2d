{-# LANGUAGE MultiParamTypeClasses #-}

module Physics.Falling2d.Rectangle
(
Rectangle(..)
, rectangleVolume
, rectangleInertiaTensor
)
where

import Data.Vect.Double.Base
import Physics.Falling.Shape.ImplicitShape
import Physics.Falling2d.InertiaTensor2d

data Rectangle = Rectangle Double Double

instance ImplicitShape Rectangle Vec2 where
  supportPoint (Vec2 dx dy) (Rectangle rx ry) = Vec2 (signum dx * rx) (signum dy * ry)

rectangleVolume :: Rectangle -> Double
rectangleVolume (Rectangle rx ry) = 4.0 * rx * ry

rectangleInertiaTensor :: Rectangle -> Double -> InertiaTensor2d
rectangleInertiaTensor (Rectangle rx ry) m = InertiaTensor2d $ 1.0 / 12.0 * m * (rx ** 2 + ry ** 2)
