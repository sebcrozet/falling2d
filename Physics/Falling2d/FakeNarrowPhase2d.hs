{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeSynonymInstances  #-}
{-# LANGUAGE FlexibleInstances     #-}
{-# LANGUAGE UndecidableInstances  #-}

module Physics.Falling2d.FakeNarrowPhase2d
(
FakeNarrowPhase2d
, fakeCollisionDispatcher
, ContactManifold2d
)
where

import Data.Vect.Double.Base
import Physics.Falling.Collision.Detection.NarrowPhase
import Physics.Falling.Collision.Collision
import Physics.Falling2d.RigidBody2d

data FakeNarrowPhase2d = FakeNarrowPhase2d

type ContactManifold2d = ContactManifold Vec2 Normal2

instance NarrowPhase FakeNarrowPhase2d (OrderedRigidBody2d idt) ContactManifold2d where
  update _ _ np   = np
  collisions _    = []
  numCollisions _ = 0

fakeCollisionDispatcher :: OrderedRigidBody2d idt -> OrderedRigidBody2d idt -> FakeNarrowPhase2d
fakeCollisionDispatcher _ _ = FakeNarrowPhase2d
