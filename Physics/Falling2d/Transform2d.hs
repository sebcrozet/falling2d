{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeSynonymInstances  #-}
{-# LANGUAGE TypeFamilies          #-}

module Physics.Falling2d.Transform2d
(
Transform2d
)
where

import Control.Monad(liftM)
import qualified Data.Vector.Generic as G
import qualified Data.Vector.Generic.Mutable as M
import Data.Vector.Unboxed
import Data.Vect.Double.Util.Dim2
import Physics.Falling.Math.Transform hiding(Vector)
import Physics.Falling2d.Vec1

type Transform2d = Proj3

instance DeltaTransform Transform2d Vec2 where
  deltaTransform p v = v .* dt
                       where
                       t  = fromProjective p
                       dt = trim t :: Mat2
  deltaTransformTranspose p v = dt *. v
                                where
                                t  = fromProjective p
                                dt = trim t :: Mat2

instance Translatable Transform2d Vec2 where
  translation p = trim t :: Vec2
                  where
                  (Mat3 _ _ t) = fromProjective p
  translate   (Vec2 x y) p = toProjectiveUnsafe newMat
                           where
                           (Mat3 r1 r2 (Vec3 x' y' _)) = fromProjective p
                           newMat = Mat3 r1 r2 (Vec3 (x + x') (y + y') 1.0)

instance Rotatable Transform2d Vec1 where
  rotation _           = undefined -- FIXME: extract the rotation from the matrix
  rotate  (Vec1 rot) p = p .*. (linear $ rotMatrix2 rot)

instance Transform       Transform2d Vec2
instance TransformSystem Transform2d Vec2 Vec1

newtype instance MVector s Vec2 = MV_Vec2 (MVector s (Double, Double))
newtype instance Vector    Vec2 = V_Vec2  (Vector    (Double, Double))
instance Unbox Vec2

instance M.MVector MVector Vec2 where
  {-# INLINE basicLength #-}
  {-# INLINE basicUnsafeSlice #-}
  {-# INLINE basicOverlaps #-}
  {-# INLINE basicUnsafeNew #-}
  {-# INLINE basicUnsafeReplicate #-}
  {-# INLINE basicUnsafeRead #-}
  {-# INLINE basicUnsafeWrite #-}
  {-# INLINE basicClear #-}
  {-# INLINE basicSet #-}
  {-# INLINE basicUnsafeCopy #-}
  {-# INLINE basicUnsafeGrow #-}
  basicLength (MV_Vec2 v)                   = M.basicLength v
  basicUnsafeSlice i n (MV_Vec2 v)          = MV_Vec2 $ M.basicUnsafeSlice i n v
  basicOverlaps (MV_Vec2 v1) (MV_Vec2 v2)   = M.basicOverlaps v1 v2
  basicUnsafeNew n                          = MV_Vec2 `liftM` M.basicUnsafeNew n
  basicUnsafeReplicate n (Vec2 x y)         = MV_Vec2 `liftM` M.basicUnsafeReplicate n (x, y)
  basicUnsafeRead (MV_Vec2 v) i             = uncurry Vec2 `liftM` M.basicUnsafeRead v i
  basicUnsafeWrite (MV_Vec2 v) i (Vec2 x y) = M.basicUnsafeWrite v i (x, y)
  basicClear (MV_Vec2 v)                    = M.basicClear v
  basicSet (MV_Vec2 v) (Vec2 x y)           = M.basicSet v (x, y)
  basicUnsafeCopy (MV_Vec2 v1) (MV_Vec2 v2) = M.basicUnsafeCopy v1 v2
  basicUnsafeGrow (MV_Vec2 v) n             = MV_Vec2 `liftM` M.basicUnsafeGrow v n

instance G.Vector Vector Vec2 where
  {-# INLINE basicUnsafeFreeze #-}
  {-# INLINE basicUnsafeThaw #-}
  {-# INLINE basicLength #-}
  {-# INLINE basicUnsafeSlice #-}
  {-# INLINE basicUnsafeIndexM #-}
  {-# INLINE elemseq #-}
  basicUnsafeFreeze (MV_Vec2 v)           = V_Vec2 `liftM` G.basicUnsafeFreeze v
  basicUnsafeThaw (V_Vec2 v)              = MV_Vec2 `liftM` G.basicUnsafeThaw v
  basicLength (V_Vec2 v)                  = G.basicLength v
  basicUnsafeSlice i n (V_Vec2 v)         = V_Vec2 $ G.basicUnsafeSlice i n v
  basicUnsafeIndexM (V_Vec2 v) i          = uncurry Vec2 `liftM` G.basicUnsafeIndexM v i
  basicUnsafeCopy (MV_Vec2 mv) (V_Vec2 v) = G.basicUnsafeCopy mv v
  elemseq _                               = seq
