{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeSynonymInstances  #-}

module Physics.Falling2d.Transform2d
(
Transform2d
)
where

import Data.Vect.Double.Base
import Data.Vect.Double.Util.Dim2
import Physics.Falling.Math.Transform
import Physics.Falling2d.Vec1

type Transform2d = Proj3

instance DeltaTransform Transform2d Vec2 where
  deltaTransform p v = dt *. v
                       where
                       t  = fromProjective p
                       dt = trim t :: Mat2

instance Translation Transform2d Vec2 where
  translation p = trim t :: Vec2
                  where
                  (Mat3 _ _ t) = fromProjective p
  translate   (Vec2 x y) p = toProjectiveUnsafe newMat
                           where
                           (Mat3 r1 r2 (Vec3 x' y' _)) = fromProjective p
                           newMat = Mat3 r1 r2 (Vec3 (x + x') (y + y') 1.0)

instance Rotation Transform2d Vec1 where
  rotate (Vec1 rotation) p = p .*. (linear $ rotMatrix2 rotation)

instance PrincipalDirections Vec2 where
  principalDirections = [ Vec2 1 0, Vec2 0 1, Vec2 (-1) 0, Vec2 0 (-1) ]

instance Transform Transform2d Vec2 Vec1
