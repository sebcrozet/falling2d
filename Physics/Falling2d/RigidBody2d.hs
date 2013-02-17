module Physics.Falling2d.RigidBody2d
(
RigidBody2d
, OrderedRigidBody2d
)
where

import Data.Vect.Double.Base
import Physics.Falling.RigidBody.RigidBody
import Physics.Falling.RigidBody.OrderedRigidBody
import Physics.Falling2d.InertiaTensor2d
import Physics.Falling2d.Vec1
import Physics.Falling2d.Shape2d

type RigidBody2d = RigidBody Proj3
                             Vec2
                             Vec1
                             InertiaTensor2d
                             InverseInertiaTensor2d
                             DynamicShape2d
                             StaticShape2d
                             TransformedDynamicShape2d
                             TransformedStaticShape2d

type OrderedRigidBody2d identifierType = OrderedRigidBody identifierType
                                                          Proj3
                                                          Vec2
                                                          Vec1
                                                          InertiaTensor2d
                                                          InverseInertiaTensor2d
                                                          DynamicShape2d
                                                          StaticShape2d
                                                          TransformedDynamicShape2d
                                                          TransformedStaticShape2d
