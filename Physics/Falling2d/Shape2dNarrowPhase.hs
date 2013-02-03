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
import Physics.Falling.Collision.Detection.IncrementalContactManifold
import Physics.Falling.Collision.Collision
import Physics.Falling2d.UnitSphere2d()
import Physics.Falling2d.Transform2d
import Physics.Falling2d.RigidBody2d
import Physics.Falling2d.Shape2d
import Physics.Falling2d.Collision2d

data Shape2dNarrowPhase = Shape2dNarrowPhase ContactManifold2d
                          deriving(Show)


instance Ord idt => NarrowPhase Shape2dNarrowPhase (OrderedRigidBody2d idt) ContactManifold2d where
  update        iorb1 iorb2 (Shape2dNarrowPhase cm) = Shape2dNarrowPhase
                                                      $ collideIndexedOrderedRigidBodies iorb1 iorb2 cm
  collisions    (Shape2dNarrowPhase cm) = cm
  numCollisions (Shape2dNarrowPhase cm) = length cm

-- rigid body <-> rigid body collision dispatch
collideIndexedOrderedRigidBodies :: Ord idt =>
                                    (Int, OrderedRigidBody2d idt) ->
                                    (Int, OrderedRigidBody2d idt) ->
                                    ContactManifold2d             ->
                                    ContactManifold2d
collideIndexedOrderedRigidBodies (id1, orb1) (id2, orb2) =
                                 collideIndexedRigidBodies id1 id2 (rigidBody orb1) (rigidBody orb2)

collideIndexedRigidBodies :: Int -> Int -> RigidBody2d -> RigidBody2d -> ContactManifold2d -> ContactManifold2d
collideIndexedRigidBodies _ id2 (StaticBody  sb) (DynamicBody db) cm = do
                          collision <- collideStaticDynamicShapes ((getCollisionVolume sb)
                                                                   , (getLocalToWorld  sb)
                                                                   , (getWorldToLocal  sb))
                                                                  ((getCollisionVolume db)
                                                                   , (getLocalToWorld  db)
                                                                   , (getWorldToLocal  db))
                                                                  (contactManifoldGeometries cm)
                          return $ collisionDescr2UnibodyCollision id2 collision
collideIndexedRigidBodies id1 id2 db@(DynamicBody _) sb@(StaticBody  _) cm = collideIndexedRigidBodies id2 id1 sb db cm
collideIndexedRigidBodies id1 id2 (DynamicBody db1) (DynamicBody db2) cm = do
                          collision <- collideDynamicDynamicShapes ((getCollisionVolume db1)
                                                                    , (getLocalToWorld  db1)
                                                                    , (getWorldToLocal  db1))
                                                                   ((getCollisionVolume db2)
                                                                    , (getLocalToWorld  db2)
                                                                    , (getWorldToLocal  db2))
                                                                  (contactManifoldGeometries cm)
                          return $ collisionDescr2BibodyCollision id1 id2 collision
collideIndexedRigidBodies _ _ (StaticBody  _) (StaticBody _) _ = error "Cannot collide two static bodies."

-- shape <-> shape collision dispatch
collideStaticDynamicShapes :: (StaticShape2d,  Transform2d, Transform2d) ->
                              (DynamicShape2d, Transform2d, Transform2d) ->
                              [ CollisionDescr2d ]                       ->
                              [ CollisionDescr2d ]
collideStaticDynamicShapes ((StaticBall2d b1), t1, _) ((Ball2d b2), t2, _) _ =
                           _maybe2List $ collideBallBall b1 b2 t1 t2

collideStaticDynamicShapes ((StaticBall2d b), t1, it1) ((Rectangle2d r), t2, it2) _ =
                            _maybe2List $ collideImplicitShapeImplicitShape (b, t1, it1)
                                                                            (r, t2, it2)
                                                                            _subdivisionNumber

collideStaticDynamicShapes ((StaticRectangle2d r), t1, it1) ((Ball2d b), t2, it2) _ =
                            _maybe2List $ collideImplicitShapeImplicitShape (r, t1, it1)
                                                                            (b, t2, it2)
                                                                            _subdivisionNumber

collideStaticDynamicShapes ((StaticRectangle2d r1), t1, it1) ((Rectangle2d r2), t2, it2) cm =
                            _updateAdd (t1, it1) (t2, it2) cm
                            $ collideImplicitShapeImplicitShape (r1, t1, it1)
                                                                (r2, t2, it2)
                                                                _subdivisionNumber

collideStaticDynamicShapes ((Plane2d p), t1, it1) ((Ball2d b), t2, it2) _ =
                           _maybe2List $ collidePlaneImplicitShape (p, t1, it1) (b, t2, it2)

collideStaticDynamicShapes ((Plane2d p), t1, it1) ((Rectangle2d r), t2, it2) cm =
                            _updateAdd (t1, it1) (t2, it2) cm
                           $ collidePlaneImplicitShape (p, t1, it1) (r, t2, it2)


collideDynamicDynamicShapes :: (DynamicShape2d, Transform2d, Transform2d) ->
                               (DynamicShape2d, Transform2d, Transform2d) ->
                               [ CollisionDescr2d ]                       ->
                               [ CollisionDescr2d ]
collideDynamicDynamicShapes ((Ball2d b1), t1, _)        ((Ball2d b2), t2, _) _ =
                            _maybe2List $ collideBallBall b1 b2 t1 t2

collideDynamicDynamicShapes ((Ball2d b), t1, it1)       ((Rectangle2d r), t2, it2) _ = 
                            _maybe2List $ collideImplicitShapeImplicitShape (b, t1, it1)
                                                                            (r, t2, it2)
                                                                            _subdivisionNumber

collideDynamicDynamicShapes ((Rectangle2d r1), t1, it1) ((Rectangle2d r2), t2, it2) cm = 
                            _updateAdd (t1, it1) (t2, it2) cm
                            $  collideImplicitShapeImplicitShape (r1, t1, it1)
                                                                 (r2, t2, it2)
                                                                 _subdivisionNumber

collideDynamicDynamicShapes s1                   s2                   cm =
                            collideDynamicDynamicShapes s2 s1 rcm >>= return . revertCollisionDescr
                            where
                            rcm = map revertCollisionDescr cm

shape2dCollisionDispatcher :: OrderedRigidBody2d idt -> OrderedRigidBody2d idt -> Shape2dNarrowPhase
shape2dCollisionDispatcher _ _ = Shape2dNarrowPhase []

_subdivisionNumber :: Int
_subdivisionNumber = 21

_maybe2List :: Maybe a -> [a]
_maybe2List Nothing  = []
_maybe2List (Just a) = [a]

_updateAdd :: (Transform2d, Transform2d) ->
              (Transform2d, Transform2d) ->
              [CollisionDescr2d]         ->
              Maybe CollisionDescr2d     ->
              [CollisionDescr2d]
_updateAdd t1s t2s cm c = case c of
           Nothing   -> updateContacts t1s t2s cm
           Just coll -> addContact coll $ updateContacts t1s t2s cm
