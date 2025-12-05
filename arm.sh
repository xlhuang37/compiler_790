# ---- Edit these once ----
export LLVM_SRC="/Users/xiaolonghuang/Documents/LLVM_ARM/llvm-project"
export SYSROOT="/Users/xiaolonghuang/Documents/LLVM_ARM/arm-gnu-toolchain-14.3.rel1/arm-none-eabi"
export IN_TREE_CLANG="/Users/xiaolonghuang/Documents/LLVM_ARM/llvm-project/build/bin/clang"
export IN_TREE_RSRC="/Users/xiaolonghuang/Documents/LLVM_ARM/llvm-project/build/lib/clang/22"

# 1) Build + "stage" compiler-rt builtins into a temp prefix
export STAGE="$HOME/stage-llvm-arm"
cmake -S "$LLVM_SRC/compiler-rt" -B "$HOME/b-crt" -G Ninja \
  -DCOMPILER_RT_BUILD_BUILTINS=ON \
  -DCMAKE_C_COMPILER="$IN_TREE_CLANG" \
  -DCMAKE_ASM_COMPILER="$IN_TREE_CLANG" \
  -DCMAKE_C_COMPILER_TARGET=arm-none-eabi \
  -DCMAKE_ASM_COMPILER_TARGET=arm-none-eabi \
  -DCMAKE_SYSROOT="$SYSROOT" \
  -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY \
  -DLLVM_CONFIG_PATH="/Users/xiaolonghuang/Documents/LLVM_ARM/llvm-project/build/bin/llvm-config" \
  -DCMAKE_INSTALL_PREFIX="$STAGE"
ninja -C "$HOME/b-crt" install

# 2) Copy ONLY the builtins runtime into your in-tree resource dir
mkdir -p "$IN_TREE_RSRC/lib/arm-unknown-none-eabi"
cp "$STAGE/lib/clang/"*/lib/arm-unknown-none-eabi/libclang_rt.builtins.a" \
   "$IN_TREE_RSRC/lib/arm-unknown-none-eabi/"

# 3) Sanity
"$IN_TREE_CLANG" --target=arm-none-eabi -print-resource-dir
ls "$IN_TREE_RSRC/lib/arm-unknown-none-eabi/libclang_rt.builtins.a"

