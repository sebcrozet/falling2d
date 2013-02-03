{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeSynonymInstances  #-}
{-# LANGUAGE FlexibleInstances     #-}
{-# LANGUAGE UndecidableInstances  #-}

module Physics.Falling2d.Shape2dNarrowPhase
(
Shape2dNarrowPhase
, shape2dCollisionDispatcher
)
where

import Data.Vect.Double.Instances()
import Physics.Falling.Collision.Detection.NarrowPhase
import Physics.Falling.RigidBody.Positionable
import Physics.Falling.RigidBody.CollisionVolume
import Physics.Falling.RigidBody.RigidBody
import Physics.Falling.RigidBody.OrderedRigidBody
import Physics.Falling.Collision.Detection.BallBallCollisionDetector
import Physics.Falling.Collision.Detection.PlaneImplicitShapeCollisionDetector
import Physics.Falling.Collision.Detection.ImplicitShapeImplicitShapeCollisionDetector
import Physics.Falling.Collision.Collision
import Physics.Falling2d.UnitSphere2d()
import Physics.Falling2d.Transform2d
import Physics.Falling2d.RigidBody2d
import Physics.Falling2d.Shape2d
import Physics.Falling2d.Collision2d

data Shape2dNarrowPhase = Shape2dNarrowPhase ContactManifold2d
                          deriving(Show)


instance Ord idt => NarrowPhase Shape2dNarrowPhase (OrderedRigidBody2d idt) ContactManifold2d where
  update        iorb1 iorb2 _           = Shape2dNarrowPhase $ collideIndexedOrderedRigidBodies iorb1 iorb2
  collisions    (Shape2dNarrowPhase cm) = cm
  numCollisions (Shape2dNarrowPhase cm) = length cm

-- rigid body <-> rigid body collision dispatch
collideIndexedOrderedRigidBodies :: Ord idt =>
                                    (Int, OrderedRigidBody2d idt) -> (Int, OrderedRigidBody2d idt) -> ContactManifold2d
collideIndexedOrderedRigidBodies (id1, orb1) (id2, orb2) =
                                 case collideIndexedRigidBodies id1 id2 (rigidBody orb1) (rigidBody orb2) of
                                   Nothing   -> []
                                   Just    c -> [c]

collideIndexedRigidBodies :: Int -> Int -> RigidBody2d -> RigidBody2d -> Maybe Collision2d
collideIndexedRigidBodies _ id2 (StaticBody  sb) (DynamicBody db) = do
                          collision <- collideStaticDynamicShapes ((getCollisionVolume sb)
                                                                   , (getLocalToWorld  sb)
                                                                   , (getWorldToLocal  sb))
                                                                  ((getCollisionVolume db)
                                                                   , (getLocalToWorld  db)
                                                                   , (getWorldToLocal  db))
                          return $ collisionDescr2UnibodyCollision id2 collision
collideIndexedRigidBodies id1 id2 db@(DynamicBody _) sb@(StaticBody  _) = collideIndexedRigidBodies id2 id1 sb db
collideIndexedRigidBodies id1 id2 (DynamicBody db1) (DynamicBody db2) = do
                          collision <- collideDynamicDynamicShapes ((getCollisionVolume db1)
                                                                    , (getLocalToWorld  db1)
                                                                    , (getWorldToLocal  db1))
                                                                   ((getCollisionVolume db2)
                                                                    , (getLocalToWorld  db2)
                                                                    , (getWorldToLocal  db2))
                          return $ collisionDescr2BibodyCollision id1 id2 collision
collideIndexedRigidBodies _ _ (StaticBody  _) (StaticBody _) = error "Cannot collide two static bodies."

-- shape <-> shape collision dispatch
collideStaticDynamicShapes :: (StaticShape2d,  Transform2d, Transform2d) ->
                              (DynamicShape2d, Transform2d, Transform2d) ->
                              Maybe CollisionDescr2d
collideStaticDynamicShapes ((StaticBall2d b1), t1, _) ((Ball2d b2), t2, _) =
                           collideBallBall b1 b2 t1 t2

collideStaticDynamicShapes ((StaticBall2d b), t1, it1) ((Rectangle2d r), t2, it2) =
                            collideImplicitShapeImplicitShape (b, t1, it1) (r, t2, it2) _subdivisionNumber

collideStaticDynamicShapes ((StaticRectangle2d r), t1, it1) ((Ball2d b), t2, it2) =
                            collideImplicitShapeImplicitShape (r, t1, it1) (b, t2, it2) _subdivisionNumber

collideStaticDynamicShapes ((StaticRectangle2d r1), t1, it1) ((Rectangle2d r2), t2, it2) =
                            collideImplicitShapeImplicitShape (r1, t1, it1) (r2, t2, it2) _subdivisionNumber

collideStaticDynamicShapes ((Plane2d p), t1, _)        ((Ball2d b), t2, _) =
                           collidePlaneImplicitShape p b t1 t2

collideStaticDynamicShapes ((Plane2d p), t1, _)        ((Rectangle2d r), t2, _) =
                           collidePlaneImplicitShape p r t1 t2


collideDynamicDynamicShapes :: (DynamicShape2d, Transform2d, Transform2d) ->
                               (DynamicShape2d, Transform2d, Transform2d) ->
                               Maybe CollisionDescr2d
collideDynamicDynamicShapes ((Ball2d b1), t1, _)        ((Ball2d b2), t2, _) =
                            collideBallBall b1 b2 t1 t2

collideDynamicDynamicShapes ((Ball2d b), t1, it1)       ((Rectangle2d r), t2, it2) = 
                            collideImplicitShapeImplicitShape (b, t1, it1) (r, t2, it2) _subdivisionNumber

collideDynamicDynamicShapes ((Rectangle2d r1), t1, it1) ((Rectangle2d r2), t2, it2) = 
                            collideImplicitShapeImplicitShape (r1, t1, it1) (r2, t2, it2) _subdivisionNumber

collideDynamicDynamicShapes s1                   s2                   =
                            collideDynamicDynamicShapes s2 s1 >>= return . revertCollisionDescr

shape2dCollisionDispatcher :: OrderedRigidBody2d idt -> OrderedRigidBody2d idt -> Shape2dNarrowPhase
shape2dCollisionDispatcher _ _ = Shape2dNarrowPhase []

_subdivisionNumber :: Int
_subdivisionNumber = 21
