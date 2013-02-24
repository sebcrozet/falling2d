{-# LANGUAGE MultiParamTypeClasses    #-}
{-# LANGUAGE TypeFamilies             #-}
{-# OPTIONS_GHC -funbox-strict-fields #-}


module Physics.Falling2d.Vec1
(
Vec1(..)
)
where

import Control.Monad(liftM)
import qualified Data.Vector.Generic as G
import qualified Data.Vector.Generic.Mutable as M
import Data.Vector.Unboxed
import Physics.Falling.Math.Transform hiding(Vector)
import qualified Physics.Falling.Math.Transform as T

data Vec1 = Vec1 !Double
            deriving(Show)

instance Eq Vec1 where
  Vec1 a == Vec1 b = a == b

instance T.Vector Vec1 where
  scalarMul s (Vec1 x) = Vec1 $ s * x
  mapVec    f (Vec1 x) = Vec1 $ f x

instance AbelianGroup Vec1 where
  (Vec1 x1) &+ (Vec1 x2) = Vec1 $ x1 + x2
  (Vec1 x1) &- (Vec1 x2) = Vec1 $ x1 - x2
  neg (Vec1 x)           = Vec1 $ -x
  zero                   = Vec1 0.0

instance DotProd Vec1 where
  (Vec1 x) &. (Vec1 x') = x * x'

instance PerpProd Vec2 Vec1 where
  (Vec2 x y) `perp` (Vec2 x' y') = Vec1 $ x * y' - y * x'

newtype instance MVector s Vec1 = MV_Vec1 (MVector s Double)
newtype instance Vector    Vec1 = V_Vec1  (Vector    Double)
instance Unbox Vec1

instance M.MVector MVector Vec1 where
  {-# INLINE basicLength          #-}
  {-# INLINE basicUnsafeSlice     #-}
  {-# INLINE basicOverlaps        #-}
  {-# INLINE basicUnsafeNew       #-}
  {-# INLINE basicUnsafeReplicate #-}
  {-# INLINE basicUnsafeRead      #-}
  {-# INLINE basicUnsafeWrite     #-}
  {-# INLINE basicClear           #-}
  {-# INLINE basicSet             #-}
  {-# INLINE basicUnsafeCopy      #-}
  {-# INLINE basicUnsafeGrow      #-}
  basicLength (MV_Vec1 v)                   = M.basicLength v
  basicUnsafeSlice i n (MV_Vec1 v)          = MV_Vec1 $ M.basicUnsafeSlice i n v
  basicOverlaps (MV_Vec1 v1) (MV_Vec1 v2)   = M.basicOverlaps v1 v2
  basicUnsafeNew n                          = MV_Vec1 `liftM` M.basicUnsafeNew n
  basicUnsafeReplicate n (Vec1 x)           = MV_Vec1 `liftM` M.basicUnsafeReplicate n x
  basicUnsafeRead (MV_Vec1 v) i             = Vec1 `liftM` M.basicUnsafeRead v i
  basicUnsafeWrite (MV_Vec1 v) i (Vec1 x)   = M.basicUnsafeWrite v i x
  basicClear (MV_Vec1 v)                    = M.basicClear v
  basicSet (MV_Vec1 v) (Vec1 x)             = M.basicSet v x
  basicUnsafeCopy (MV_Vec1 v1) (MV_Vec1 v2) = M.basicUnsafeCopy v1 v2
  basicUnsafeGrow (MV_Vec1 v) n             = MV_Vec1 `liftM` M.basicUnsafeGrow v n

instance G.Vector Vector Vec1 where
  {-# INLINE basicUnsafeFreeze #-}
  {-# INLINE basicUnsafeThaw   #-}
  {-# INLINE basicLength       #-}
  {-# INLINE basicUnsafeSlice  #-}
  {-# INLINE basicUnsafeIndexM #-}
  {-# INLINE elemseq           #-}
  basicUnsafeFreeze (MV_Vec1 v)           = V_Vec1 `liftM` G.basicUnsafeFreeze v
  basicUnsafeThaw (V_Vec1 v)              = MV_Vec1 `liftM` G.basicUnsafeThaw v
  basicLength (V_Vec1 v)                  = G.basicLength v
  basicUnsafeSlice i n (V_Vec1 v)         = V_Vec1 $ G.basicUnsafeSlice i n v
  basicUnsafeIndexM (V_Vec1 v) i          = Vec1 `liftM` G.basicUnsafeIndexM v i
  basicUnsafeCopy (MV_Vec1 mv) (V_Vec1 v) = G.basicUnsafeCopy mv v
  elemseq _                               = seq
