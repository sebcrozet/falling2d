module Physics.Falling2d.World2d
(
World2d
, DefaultWorld2d
, mkWorld2d
)
where

import Data.Vect.Double.Base
import qualified Physics.Falling.World.WorldWithRigidBody as W
import Physics.Falling.Collision.Detection.BruteForceBroadPhase
import Physics.Falling.RigidBody.RigidBodySolver
import Physics.Falling2d.FakeNarrowPhase2d
import Physics.Falling2d.RigidBody2d
import Physics.Falling2d.InertiaTensor2d
import Physics.Falling2d.Vec1
import Physics.Falling2d.Transform2d()
import Physics.Falling2d.Shape2d

type World2d identifierType broadPhaseType narrowPhaseType contactManifoldType = W.World Proj3
                                                                                         Vec2
                                                                                         Vec1
                                                                                         InertiaTensor2d
                                                                                         InverseInertiaTensor2d
                                                                                         DynamicShape2d
                                                                                         StaticShape2d
                                                                                         broadPhaseType
                                                                                         narrowPhaseType
                                                                                         contactManifoldType
                                                                                         identifierType

type DefaultWorld2d identifierType = World2d identifierType
                                             (BruteForceBroadPhase (OrderedRigidBody2d identifierType))
                                             FakeNarrowPhase2d
                                             ContactManifold2d

-- mkWorld :: World identifierType fixme1 fixme2 fixme3
-- mkWorld2d :: a -> World2d identifierType
-- mkWorld2d = W.mkWorld (mkBroadPhase :: BruteForceBroadPhase (OrderedRigidBody2d identifierType)) undefined

mkWorld2d :: (Ord identifierType) => DefaultWorld2d identifierType
mkWorld2d = W.mkWorld mkBroadPhase
                      fakeCollisionDispatcher
                      solveConstraintsIsland
