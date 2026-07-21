#!/usr/bin/env bash
set -euo pipefail

OUTDIR="${1:?Usage: $0 <output-directory>}"
mkdir -p "$OUTDIR"
LOG="$OUTDIR/toolchain_preflight.log"
exec > >(tee -i "$LOG") 2>&1

echo "=== Toolchain Preflight ==="
echo "Timestamp: $(date -Iseconds)"
echo "Hostname:  $(hostname)"
echo "Kernel:    $(uname -r)"

echo "=== Compiler Paths ==="
for cmd in gcc g++ gcc-11 g++-11 cc c++ nvcc; do
  p=$(command -v "$cmd" 2>/dev/null || echo "NOT_FOUND")
  echo "$cmd: $p"
done

echo "=== GCC/cc1plus Discovery ==="
for compiler in gcc g++ gcc-11 g++-11; do
  if ! command -v "$compiler" >/dev/null 2>&1; then
    echo "$compiler: NOT_FOUND"
    continue
  fi
  echo "--- $compiler ---"
  "$compiler" --version | head -n 1 || true
  "$compiler" -print-search-dirs 2>&1 || true
  cc1plus_path="$("$compiler" -print-prog-name=cc1plus 2>/dev/null || true)"
  echo "$compiler -print-prog-name=cc1plus: ${cc1plus_path:-<empty>}"
  if [ -n "$cc1plus_path" ]; then
    ls -l "$cc1plus_path" 2>&1 || true
    file "$cc1plus_path" 2>&1 || true
    ldd "$cc1plus_path" 2>&1 || true
  fi
done

echo "=== NVCC Version ==="
nvcc --version 2>&1 || echo "nvcc not available"

echo "=== Key Environment ==="
for var in PATH CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH LIBRARY_PATH LD_LIBRARY_PATH CUDA_HOME CUDA_PATH CUDAHOSTCXX CC CXX CUDACXX COMPILER_PATH GCC_EXEC_PREFIX PYTHONHOME PYTHONPATH CONDA_PREFIX VIRTUAL_ENV; do
  echo "${var}=${!var:-<unset>}"
done

echo "=== Env Sorted ==="
env | sort

echo "=== C++ Probe ==="
cat > /tmp/cc1plus_probe.cpp <<'EOF'
#include <iostream>
int main() { std::cout << "cc1plus probe" << std::endl; return 0; }
EOF
g++ /tmp/cc1plus_probe.cpp -o /tmp/cc1plus_probe
/tmp/cc1plus_probe

echo "=== CUDA Probe ==="
cat > /tmp/nvcc_probe.cu <<'EOF'
#include <cstdio>
__global__ void kernel() {}
int main() { kernel<<<1, 1>>>(); std::puts("nvcc probe"); return 0; }
EOF
if command -v nvcc &>/dev/null; then
  set +e
  nvcc -v /tmp/nvcc_probe.cu -o /tmp/nvcc_probe
  nvcc_rc=$?
  set -e
  echo "nvcc_probe_exit_code=$nvcc_rc"
  if [ "$nvcc_rc" -eq 0 ]; then
    /tmp/nvcc_probe
  else
    echo "CUDA probe FAILED"
  fi
else
  echo "CUDA probe SKIPPED (no nvcc)"
fi

echo "=== Preflight Complete ==="
