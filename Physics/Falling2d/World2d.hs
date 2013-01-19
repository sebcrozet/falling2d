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
import Physics.Falling2d.RigidBody2d
import Physics.Falling2d.InertiaTensor2d
import Physics.Falling2d.Vec1
import Physics.Falling2d.Transform2d
import Physics.Falling2d.Shape2d
-- import Physics.Falling2d.FakeNarrowPhase2d
import Physics.Falling2d.Shape2dNarrowPhase
import Physics.Falling2d.Collision2d

type World2d identifierType broadPhaseType narrowPhaseType contactManifoldType = W.World Transform2d
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
                                             Shape2dNarrowPhase
                                             ContactManifold2d

mkWorld2d :: (Ord identifierType) => DefaultWorld2d identifierType
mkWorld2d = W.mkWorld mkBroadPhase
                      shape2dCollisionDispatcher
                      solveConstraintsIsland
