{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeSynonymInstances  #-}

module Physics.Falling2d.Shape2d
(
DynamicShape2d(..)
, StaticShape2d(..)
, TransformedDynamicShape2d(..)
, TransformedStaticShape2d(..)
)
where

import Data.Vect.Double.Base
import Physics.Falling.Shape.VolumetricShape
import Physics.Falling.Shape.TransformableShape
import Physics.Falling.Shape.TransformedShape
import Physics.Falling.Shape.Ball
import Physics.Falling.Shape.Plane
import Physics.Falling2d.Rectangle
import Physics.Falling2d.Vec1
import Physics.Falling2d.Transform2d
import Physics.Falling2d.InertiaTensor2d

data DynamicShape2d = Ball2d               (Ball Vec2)
                      | Rectangle2d        Rectangle
                      | CompoundShape2d    [ DynamicShape2d ]
                      | TransformedShape2d (TransformedShape DynamicShape2d Transform2d Vec2)
                      deriving(Show)

data StaticShape2d  = StaticBall2d               (Ball Vec2)
                      | StaticRectangle2d        Rectangle
                      | Plane2d                  (Plane Vec2 Normal2)
                      | CompoundStaticShape2d    [ StaticShape2d ]
                      | TransformedStaticShape2d (TransformedShape StaticShape2d Transform2d Vec2)
                      deriving(Show)

data TransformedDynamicShape2d = TransformedBall2d        (Ball Vec2)
                                 | TransformedRectangle2d (TransformedShape Rectangle Transform2d Vec2)
                                 | TransformedCompound2d  [ TransformedDynamicShape2d ]
                                 deriving(Show)

data TransformedStaticShape2d = TransformedStaticBall2d        (Ball Vec2)
                                | TransformedPlane2d           (Plane Vec2 Normal2)
                                | TransformedStaticRectangle2d (TransformedShape Rectangle Transform2d Vec2)
                                | TransformedStaticCompound2d  [ TransformedStaticShape2d ]
                                deriving(Show)

instance VolumetricShape DynamicShape2d InertiaTensor2d InverseInertiaTensor2d Vec1 Proj3 where
  volume                   (Ball2d b)               = ballVolume b 2
  volume                   (Rectangle2d r)          = rectangleVolume r
  volume                   (TransformedShape2d s)   = volume s
  volume                   (CompoundShape2d cs)     = sum $ map volume cs
  objectFrameInertiaTensor (Ball2d (Ball  _ r))   m = InertiaTensor2d $ m * r * r / 2.0
  objectFrameInertiaTensor (Rectangle2d r)        m = rectangleInertiaTensor r m
  objectFrameInertiaTensor (TransformedShape2d s) m = objectFrameInertiaTensor s m -- FIXME: take the transform in account
  objectFrameInertiaTensor (CompoundShape2d _)    _ = undefined                    -- FIXME: «sum» the inertia tensors

instance TransformableShape DynamicShape2d Transform2d TransformedDynamicShape2d where
  transformShape t (Ball2d b)              = TransformedBall2d $ transformShape t b
  transformShape t (TransformedShape2d ts) = transformShape t ts
  transformShape t (Rectangle2d r)         = TransformedRectangle2d $ TransformedShape r t 
  transformShape t (CompoundShape2d cs)    = TransformedCompound2d $ map (transformShape t) cs

instance TransformableShape StaticShape2d  Transform2d TransformedStaticShape2d  where
  transformShape t (StaticBall2d b)              = TransformedStaticBall2d  $ transformShape t b
  transformShape t (StaticRectangle2d r)         = TransformedStaticRectangle2d $ TransformedShape r t
  transformShape t (Plane2d p)                   = TransformedPlane2d $ transformShape t p
  transformShape t (TransformedStaticShape2d ts) = transformShape t ts
  transformShape t (CompoundStaticShape2d cs)    = TransformedStaticCompound2d $ map (transformShape t) cs
