{-# LANGUAGE MultiParamTypeClasses #-}

module Physics.Falling2d.OrthonormalBasis2d
(
)
where

import Data.Vect.Double.Base
import Physics.Falling.Math.OrthonormalBasis

instance OrthonormalBasis Vec2 Normal2 where
  canonicalBasis  = [ toNormalUnsafe $ Vec2 1.0 0.0, toNormalUnsafe $ Vec2 0.0 1.0 ]
  completeBasis n = let Vec2 x y = fromNormal n in
                    (n, [ toNormalUnsafe $ Vec2 (-y) x ])
