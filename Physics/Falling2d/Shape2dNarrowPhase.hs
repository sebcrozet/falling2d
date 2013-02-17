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
import Control.Monad
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
import Physics.Falling2d.OrthonormalBasis2d()
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
                          collision <- collideStaticDynamicShapes ((worldSpaceCollisionVolume sb)
                                                                   , (localToWorld  sb)
                                                                   , (worldToLocal  sb))
                                                                  ((worldSpaceCollisionVolume db)
                                                                   , (localToWorld  db)
                                                                   , (worldToLocal  db))
                                                                  (contactManifoldGeometries cm)
                          return $ collisionDescr2UnibodyCollision id2 collision
collideIndexedRigidBodies id1 id2 db@(DynamicBody _) sb@(StaticBody  _) cm = collideIndexedRigidBodies id2 id1 sb db cm
collideIndexedRigidBodies id1 id2 (DynamicBody db1) (DynamicBody db2) cm =
                          do
                          collision <- collideDynamicDynamicShapes ((worldSpaceCollisionVolume db1)
                                                                    , (localToWorld  db1)
                                                                    , (worldToLocal  db1))
                                                                   ((worldSpaceCollisionVolume db2)
                                                                    , (localToWorld  db2)
                                                                    , (worldToLocal  db2))
                                                                  (contactManifoldGeometries cm)
                          return $ collisionDescr2BibodyCollision id1 id2 collision
collideIndexedRigidBodies _ _ (StaticBody  _) (StaticBody _) _ = error "Cannot collide two static bodies."

-- shape <-> shape collision dispatch
collideStaticDynamicShapes :: (TransformedStaticShape2d,  Transform2d, Transform2d) ->
                              (TransformedDynamicShape2d, Transform2d, Transform2d) ->
                              [ CollisionDescr2d ]                       ->
                              [ CollisionDescr2d ]
collideStaticDynamicShapes ((TransformedStaticBall2d b1), _, it1) ((TransformedBall2d b2), _, it2) _ =
                           _maybe2List
                           $ liftM (mkCollisionDescr it1 it2)
                           $ collideBallBall b1 b2

collideStaticDynamicShapes ((TransformedStaticBall2d b), _, it1) ((TransformedRectangle2d r), _, it2) _ =
                            _maybe2List
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideImplicitShapeImplicitShape b r _subdivisionNumber

collideStaticDynamicShapes ((TransformedStaticRectangle2d r), _, it1) ((TransformedBall2d b), _, it2) _ =
                            _maybe2List
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideImplicitShapeImplicitShape r b _subdivisionNumber

collideStaticDynamicShapes ((TransformedStaticRectangle2d r1), t1, it1) ((TransformedRectangle2d r2), t2, it2) cm =
                            _updateAdd t1 t2 cm
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideImplicitShapeImplicitShape r1 r2 _subdivisionNumber

collideStaticDynamicShapes ((TransformedPlane2d p), _, it1) ((TransformedBall2d b), _, it2) _ =
                           _maybe2List
                           $ liftM (mkCollisionDescr it1 it2)
                           $ collidePlaneImplicitShape p b

collideStaticDynamicShapes ((TransformedPlane2d p), t1, it1) ((TransformedRectangle2d r), t2, it2) cm =
                            _updateAdd t1 t2 cm
                           $ liftM (mkCollisionDescr it1 it2)
                           $ collidePlaneImplicitShape p r


collideDynamicDynamicShapes :: (TransformedDynamicShape2d, Transform2d, Transform2d) ->
                               (TransformedDynamicShape2d, Transform2d, Transform2d) ->
                               [ CollisionDescr2d ]                       ->
                               [ CollisionDescr2d ]
collideDynamicDynamicShapes ((TransformedBall2d b1), _, it1)        ((TransformedBall2d b2), _, it2) _ =
                            _maybe2List
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideBallBall b1 b2

collideDynamicDynamicShapes ((TransformedRectangle2d r), _, it1) ((TransformedBall2d b), _, it2) _ = 
                            _maybe2List
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideImplicitShapeImplicitShape r b _subdivisionNumber

collideDynamicDynamicShapes ((TransformedRectangle2d r1), t1, it1) ((TransformedRectangle2d r2), t2, it2) cm = 
                            _updateAdd t1 t2 cm
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideImplicitShapeImplicitShape r1 r2 _subdivisionNumber

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

_updateAdd :: Transform2d -> Transform2d -> [CollisionDescr2d] -> Maybe CollisionDescr2d -> [CollisionDescr2d]
_updateAdd t1 t2 cm c = case c of
           Nothing   -> updateContacts t1 t2 cm
           Just coll -> addContact coll $ updateContacts t1 t2 cm
