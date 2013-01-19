module Physics.Falling2d.Collision2d
(
CollisionDescr2d
, Collision2d
, ContactManifold2d
)
where

import Data.Vect.Double.Base
import Physics.Falling.Collision.Collision

type CollisionDescr2d  = CollisionDescr  Vec2 Normal2
type Collision2d       = Collision       Vec2 Normal2
type ContactManifold2d = ContactManifold Vec2 Normal2
