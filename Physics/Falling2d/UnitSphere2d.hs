module Physics.Falling2d.UnitSphere2d
(
)
where

import System.Random
import Data.Random.Normal
import Data.Vect.Double.Base
import Physics.Falling.Math.UnitSphere

instance UnitSphere Normal2 where
  unitSphereSamples    = _randomCirclePoints 
  nUnitSphereSamples n = map (\ang -> toNormalUnsafe $ Vec2 (cos ang) (sin ang)) subdivs
                         where
                         subdivs = [ fromIntegral i * 2.0 * pi / fromIntegral n | i <- [ 0 .. n - 1 ] ]

_randomCirclePoints :: [Normal2]
_randomCirclePoints  = map mkNormal $ _doubleList2Vec2List $ normals $ mkStdGen 0

_doubleList2Vec2List :: [Double] -> [Vec2]
_doubleList2Vec2List (a:b:l) = Vec2 a b : _doubleList2Vec2List l
_doubleList2Vec2List _       = []
