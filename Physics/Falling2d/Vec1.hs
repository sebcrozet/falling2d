{-# LANGUAGE MultiParamTypeClasses #-}

module Physics.Falling2d.Vec1
(
Vec1(..)
)
where

import Data.Vect.Double.Base
import Physics.Falling.Math.Transform

data Vec1 = Vec1 !Double
            deriving(Show)

instance Eq Vec1 where
  Vec1 a == Vec1 b = a == b

instance Vector       Vec1 where  
  scalarMul s (Vec1 x) = Vec1 $ s * x
  mapVec    f (Vec1 x) = Vec1 $ f x

instance AbelianGroup Vec1 where  
  (Vec1 x1) &+ (Vec1 x2) = Vec1 $ x1 + x2
  (Vec1 x1) &- (Vec1 x2) = Vec1 $ x1 - x2
  neg (Vec1 x)           = Vec1 $ -x
  zero                   = Vec1 0.0

instance DotProd Vec1 where
  (Vec1 x) &. (Vec1 x') = x * x'

instance PerpProd Vec2 Vec1 where
  (Vec2 x y) `perp` (Vec2 x' y') = Vec1 $ x * y' - y * x'
