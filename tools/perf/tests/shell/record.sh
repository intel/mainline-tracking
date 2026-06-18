#!/bin/bash
# perf record tests (exclusive)
# SPDX-License-Identifier: GPL-2.0

set -e

shelldir=$(dirname "$0")
# shellcheck source=lib/waiting.sh
. "${shelldir}"/lib/waiting.sh

# shellcheck source=lib/perf_has_symbol.sh
. "${shelldir}"/lib/perf_has_symbol.sh

testsym="test_loop"
testsym2="brstack"

skip_test_missing_symbol ${testsym}
skip_test_missing_symbol ${testsym2}

err=0
perfdata=$(mktemp /tmp/__perf_test.perf.data.XXXXX)
script_output=$(mktemp /tmp/__perf_test.perf.data.XXXXX.script)
testprog="perf test -w thloop"
cpu_pmu_dir="/sys/bus/event_source/devices/cpu*"
br_cntr_file="/caps/branch_counter_nr"
br_cntr_output="branch stack counters"
br_cntr_script_output="br_cntr: A"

default_fd_limit=$(ulimit -Sn)
# With option --threads=cpu the number of open file descriptors should be
# equal to sum of:    nmb_cpus * nmb_events (2+dummy),
#                     nmb_threads for perf.data.n (equal to nmb_cpus) and
#                     2*nmb_cpus of pipes = 4*nmb_cpus (each pipe has 2 ends)
# All together it needs 8*nmb_cpus file descriptors plus some are also used
# outside of testing, thus raising the limit to 16*nmb_cpus
min_fd_limit=$(($(getconf _NPROCESSORS_ONLN) * 16))

cleanup() {
  rm -f "${perfdata}"
  rm -f "${perfdata}".old
  rm -f "${script_output}"

  trap - EXIT TERM INT
}

trap_cleanup() {
  echo "Unexpected signal in ${FUNCNAME[1]}"
  cleanup
  exit 1
}
trap trap_cleanup EXIT TERM INT

test_per_thread() {
  echo "Basic --per-thread mode test"
  if ! perf record -o /dev/null --quiet ${testprog} 2> /dev/null
  then
    echo "Per-thread record [Skipped event not supported]"
    return
  fi
  if ! perf record --per-thread -o "${perfdata}" ${testprog} 2> /dev/null
  then
    echo "Per-thread record [Failed record]"
    err=1
    return
  fi
  if ! perf report -i "${perfdata}" -q | grep -q "${testsym}"
  then
    echo "Per-thread record [Failed missing output]"
    err=1
    return
  fi

  # run the test program in background (for 30 seconds)
  ${testprog} 30 &
  TESTPID=$!

  rm -f "${perfdata}"

  wait_for_threads ${TESTPID} 2
  perf record -p "${TESTPID}" --per-thread -o "${perfdata}" sleep 1 2> /dev/null
  kill ${TESTPID}

  if [ ! -e "${perfdata}" ]
  then
    echo "Per-thread record [Failed record -p]"
    err=1
    return
  fi
  if ! perf report -i "${perfdata}" -q | grep -q "${testsym}"
  then
    echo "Per-thread record [Failed -p missing output]"
    err=1
    return
  fi

  echo "Basic --per-thread mode test [Success]"
}

test_register_capture() {
  echo "Register capture test"
  if ! perf list pmu | grep -q 'br_inst_retired.near_call'
  then
    echo "Register capture test [Skipped missing event]"
    return
  fi
  if ! perf record --intr-regs=\? 2>&1 | grep -q 'available registers: AX BX CX DX SI DI BP SP IP FLAGS CS SS R8 R9 R10 R11 R12 R13 R14 R15'
  then
    echo "Register capture test [Skipped missing registers]"
    return
  fi
  if ! perf record -o - --intr-regs=di,r8,dx,cx -e br_inst_retired.near_call \
    -c 1000 --per-thread ${testprog} 2> /dev/null \
    | perf script -F ip,sym,iregs -i - 2> /dev/null \
    | grep -q "DI:"
  then
    echo "Register capture test [Failed missing output]"
    err=1
    return
  fi
  echo "Register capture test [Success]"
}

has_required_regs() {
  local regs_output="$1"
  shift

  for reg in "$@"
  do
    if ! echo "${regs_output}" | grep -q -i "${reg}"
    then
      return 1
    fi
  done

  return 0
}

validate_gp_reg_sampling() {
  local regs_opt="$1"
  local regs_value="$2"
  local script_field="$3"
  local sample_output
  shift 3

  if ! sample_output=$(perf record -o - "${regs_opt}=${regs_value}" \
    -e br_inst_retired.near_call -c 1000 --per-thread ${testprog} 2> /dev/null \
    | perf script -F ip,sym,"${script_field}" -i - 2> /dev/null)
  then
    return 1
  fi

  if ! has_required_regs "${sample_output}" "$@"
  then
    return 1
  fi

  return 0
}

test_egpr_register_capture() {
  local arch
  local intr_regs
  local user_regs
  local tested=0

  echo "eGPR/SSP register capture test"

  arch=$(uname -m)
  case ${arch} in
  x86_64|i386)
    ;;
  *)
    echo "eGPR/SSP register capture test [Skipped non-x86 platform]"
    return
    ;;
  esac

  if ! perf list pmu | grep -q 'br_inst_retired.near_call'
  then
    echo "eGPR/SSP register capture test [Skipped missing event]"
    return
  fi

  intr_regs=$(perf record --intr-regs=\? 2>&1 || true)
  user_regs=$(perf record --user-regs=\? 2>&1 || true)

  if has_required_regs "${intr_regs}" R16 R31
  then
    if ! validate_gp_reg_sampling "--intr-regs" "r16,r31" "iregs" "R16:" "R31:"
    then
      echo "eGPR/SSP register capture test [Failed eGPR intr-regs validation]"
      err=1
      return
    fi
    tested=1
  else
    echo "eGPR/SSP register capture test [Skipped missing eGPR intr-regs (R16/R31)]"
  fi

  if has_required_regs "${user_regs}" R16 R31
  then
    if ! validate_gp_reg_sampling "--user-regs" "r16,r31" "uregs" "R16:" "R31:"
    then
      echo "eGPR/SSP register capture test [Failed eGPR user-regs validation]"
      err=1
      return
    fi
    tested=1
  else
    echo "eGPR/SSP register capture test [Skipped missing eGPR user-regs (R16/R31)]"
  fi

  if has_required_regs "${intr_regs}" SSP
  then
    if ! validate_gp_reg_sampling "--intr-regs" "ssp" "iregs" "SSP:"
    then
      echo "eGPR/SSP register capture test [Failed SSP intr-regs validation]"
      err=1
      return
    fi
    tested=1
  else
    echo "eGPR/SSP register capture test [Skipped missing SSP intr-regs]"
  fi

  if has_required_regs "${user_regs}" SSP
  then
    if ! validate_gp_reg_sampling "--user-regs" "ssp" "uregs" "SSP:"
    then
      echo "eGPR/SSP register capture test [Failed SSP user-regs validation]"
      err=1
      return
    fi
    tested=1
  else
    echo "eGPR/SSP register capture test [Skipped missing SSP user-regs]"
  fi

  if [ ${tested} -eq 0 ]
  then
    echo "eGPR/SSP register capture test [Skipped missing eGPR/SSP registers]"
    return
  fi

  echo "eGPR/SSP register capture test [Success]"
}

extract_x86_advertised_simd_classes() {
  local regs_output="$1"

  echo "${regs_output}" \
    | grep -oE '(ZMM|YMM|XMM|OPMASK)[0-9]+-[0-9]+' \
    | sed -E 's/[0-9]+-[0-9]+$//' \
    | tr '[:upper:]' '[:lower:]' \
    | sort -u
}

ordered_x86_simd_classes() {
  local advertised_classes="$1"
  local simd_classes=""

  if echo "${advertised_classes}" | grep -qw zmm
  then
    simd_classes="${simd_classes} zmm"
  fi
  if echo "${advertised_classes}" | grep -qw ymm
  then
    simd_classes="${simd_classes} ymm"
  fi
  if echo "${advertised_classes}" | grep -qw xmm
  then
    simd_classes="${simd_classes} xmm"
  fi
  if echo "${advertised_classes}" | grep -qw opmask
  then
    simd_classes="${simd_classes} opmask"
  fi

  echo "${simd_classes}" | xargs
}

validate_simd_sampling_mode() {
  local regs_opt="$1"
  local simd_classes="$2"
  local script_field="$3"
  local simd_class
  local sample_output

  for simd_class in ${simd_classes}
  do
    if ! sample_output=$(perf record -o - "${regs_opt}=${simd_class}" \
      -e br_inst_retired.near_call -c 1000 --per-thread ${testprog} 2> /dev/null \
      | perf script -F ip,sym,"${script_field}" -i - 2> /dev/null)
    then
      echo "SIMD register capture test [Failed record ${regs_opt}=${simd_class}]"
      return 1
    fi

    if ! has_required_regs "${sample_output}" "${simd_class}"
    then
      echo "SIMD register capture test [Failed missing ${simd_class} sampling data]"
      return 1
    fi
  done

  return 0
}

test_simd_register_capture() {
  local arch
  local intr_regs
  local user_regs
  local simd_classes
  local user_simd_classes
  local advertised_intr_classes
  local advertised_user_classes
  local tested=0

  echo "SIMD register capture test"
  if ! perf list pmu | grep -q 'br_inst_retired.near_call'
  then
    echo "SIMD register capture test [Skipped missing event]"
    return
  fi

  intr_regs=$(perf record --intr-regs=\? 2>&1 || true)
  user_regs=$(perf record --user-regs=\? 2>&1 || true)

  simd_classes=""
  user_simd_classes=""

  arch=$(uname -m)
  case ${arch} in
  x86_64|i386)
    advertised_intr_classes=$(extract_x86_advertised_simd_classes "${intr_regs}")
    advertised_user_classes=$(extract_x86_advertised_simd_classes "${user_regs}")

    simd_classes=$(ordered_x86_simd_classes "${advertised_intr_classes}")
    user_simd_classes=$(ordered_x86_simd_classes "${advertised_user_classes}")
    ;;
  *)
    ;;
  esac

  if [ -z "${simd_classes}" ]
  then
    echo "SIMD register capture test [Skipped missing intr SIMD registers]"
  elif ! validate_simd_sampling_mode "--intr-regs" "${simd_classes}" "iregs"
  then
    echo "SIMD register capture test [Failed missing intr SIMD register sampling data]"
    err=1
    return
  else
    tested=1
  fi

  if [ -z "${user_simd_classes}" ]
  then
    echo "SIMD register capture test [Skipped missing user SIMD registers]"
  elif ! validate_simd_sampling_mode "--user-regs" "${user_simd_classes}" "uregs"
  then
    echo "SIMD register capture test [Failed missing user SIMD register sampling data]"
    err=1
    return
  else
    tested=1
  fi

  if [ ${tested} -eq 0 ]
  then
    echo "SIMD register capture test [Skipped missing SIMD registers]"
    return
  fi

  echo "SIMD register capture test [Success]"
}

test_system_wide() {
  echo "Basic --system-wide mode test"
  if ! perf record -aB --synth=no -o "${perfdata}" ${testprog} 2> /dev/null
  then
    echo "System-wide record [Skipped not supported]"
    return
  fi
  if ! perf report -i "${perfdata}" -q | grep -q "${testsym}"
  then
    echo "System-wide record [Failed missing output]"
    err=1
    return
  fi
  if ! perf record -aB --synth=no -e cpu-clock,cs --threads=cpu \
    -o "${perfdata}" ${testprog} 2> /dev/null
  then
    echo "System-wide record [Failed record --threads option]"
    err=1
    return
  fi
  if ! perf report -i "${perfdata}" -q | grep -q "${testsym}"
  then
    echo "System-wide record [Failed --threads missing output]"
    err=1
    return
  fi
  echo "Basic --system-wide mode test [Success]"
}

test_workload() {
  echo "Basic target workload test"
  if ! perf record -o "${perfdata}" ${testprog} 2> /dev/null
  then
    echo "Workload record [Failed record]"
    err=1
    return
  fi
  if ! perf report -i "${perfdata}" -q | grep -q "${testsym}"
  then
    echo "Workload record [Failed missing output]"
    err=1
    return
  fi
  if ! perf record -e cpu-clock,cs --threads=package \
    -o "${perfdata}" ${testprog} 2> /dev/null
  then
    echo "Workload record [Failed record --threads option]"
    err=1
    return
  fi
  if ! perf report -i "${perfdata}" -q | grep -q "${testsym}"
  then
    echo "Workload record [Failed --threads missing output]"
    err=1
    return
  fi
  echo "Basic target workload test [Success]"
}

test_branch_counter() {
  echo "Branch counter test"
  # Check if the branch counter feature is supported
  for dir in $cpu_pmu_dir
  do
    if [ ! -e "$dir$br_cntr_file" ]
    then
      echo "branch counter feature not supported on all core PMUs ($dir) [Skipped]"
      return
    fi
  done
  if ! perf record -o "${perfdata}" -e "{branches:p,instructions}" -j any,counter ${testprog} 2> /dev/null
  then
    echo "Branch counter record test [Failed record]"
    err=1
    return
  fi
  if ! perf report -i "${perfdata}" -D -q | grep -q "$br_cntr_output"
  then
    echo "Branch counter report test [Failed missing output]"
    err=1
    return
  fi
  if ! perf script -i "${perfdata}" -F +brstackinsn,+brcntr | grep -q "$br_cntr_script_output"
  then
    echo " Branch counter script test [Failed missing output]"
    err=1
    return
  fi
  echo "Branch counter test [Success]"
}

test_cgroup() {
  echo "Cgroup sampling test"
  if ! perf record -aB --synth=cgroup --all-cgroups -o "${perfdata}" ${testprog} 2> /dev/null
  then
    echo "Cgroup sampling [Skipped not supported]"
    return
  fi
  if ! perf report -i "${perfdata}" -D | grep -q "CGROUP"
  then
    echo "Cgroup sampling [Failed missing output]"
    err=1
    return
  fi
  if ! perf script -i "${perfdata}" -F cgroup | grep -q -v "unknown"
  then
    echo "Cgroup sampling [Failed cannot resolve cgroup names]"
    err=1
    return
  fi
  echo "Cgroup sampling test [Success]"
}

test_uid() {
  echo "Uid sampling test"
  if ! perf record -aB --synth=no --uid "$(id -u)" -o "${perfdata}" ${testprog} \
    > "${script_output}" 2>&1
  then
    if grep -q "libbpf.*EPERM" "${script_output}"
    then
      echo "Uid sampling [Skipped permissions]"
      return
    else
      echo "Uid sampling [Failed to record]"
      err=1
      # cat "${script_output}"
      return
    fi
  fi
  if ! perf report -i "${perfdata}" -q | grep -q "${testsym}"
  then
    echo "Uid sampling [Failed missing output]"
    err=1
    return
  fi
  echo "Uid sampling test [Success]"
}

test_leader_sampling() {
  echo "Basic leader sampling test"
  events="{cycles,cycles}:Su"
  [ "$(uname -m)" = "s390x" ] && {
    [ ! -d /sys/devices/cpum_sf ] && {
      echo "No CPUMF [Skipped record]"
      return
    }
    events="{cpum_sf/SF_CYCLES_BASIC/,cycles}:Su"
    perf record -o "${perfdata}" -e "$events" -- perf test -w brstack 2> /dev/null
    # Perf grouping might be unsupported, depends on version.
    [ "$?" -ne 0 ] && {
      echo "Grouping not support [Skipped record]"
      return
    }
  }
  if ! perf record -o "${perfdata}" -e "$events" -- \
    perf test -w brstack 2> /dev/null
  then
    echo "Leader sampling [Failed record]"
    err=1
    return
  fi
  perf script -i "${perfdata}" | grep brstack > $script_output
  # Check if the two instruction counts are equal in each record.
  # However, the throttling code doesn't consider event grouping. During throttling, only the
  # leader is stopped, causing the slave's counts significantly higher. To temporarily solve this,
  # let's set the tolerance rate to 80%.
  # TODO: Revert the code for tolerance once the throttling mechanism is fixed.
  index=0
  valid_counts=0
  invalid_counts=0
  tolerance_rate=0.8
  while IFS= read -r line
  do
    cycles=$(echo $line | awk '{for(i=1;i<=NF;i++) if($i=="cycles:") print $(i-1)}')
    if [ $(($index%2)) -ne 0 ] && [ ${cycles}x != ${prev_cycles}x ]
    then
      invalid_counts=$(($invalid_counts+1))
    else
      valid_counts=$(($valid_counts+1))
    fi
    index=$(($index+1))
    prev_cycles=$cycles
  done < "${script_output}"
  total_counts=$(bc <<< "$invalid_counts+$valid_counts")
  if (( $(bc <<< "$total_counts <= 0") ))
  then
    echo "Leader sampling [No sample generated]"
    err=1
    return
  fi
  isok=$(bc <<< "scale=2; if (($invalid_counts/$total_counts) < (1-$tolerance_rate)) { 0 } else { 1 };")
  if [ $isok -eq 1 ]
  then
     echo "Leader sampling [Failed inconsistent cycles count]"
     err=1
  else
    echo "Basic leader sampling test [Success]"
  fi
}

test_topdown_leader_sampling() {
  echo "Topdown leader sampling test"
  if ! perf stat -e "{slots,topdown-retiring}" true 2> /dev/null
  then
    echo "Topdown leader sampling [Skipped event parsing failed]"
    return
  fi
  if ! perf record -o "${perfdata}" -e "{instructions,slots,topdown-retiring}:S" true 2> /dev/null
  then
    echo "Topdown leader sampling [Failed topdown events not reordered correctly]"
    err=1
    return
  fi
  echo "Topdown leader sampling test [Success]"
}

test_precise_max() {
  local -i skipped=0

  echo "precise_max attribute test"
  # Just to make sure event cycles is supported for sampling
  if perf record -o "${perfdata}" -e "cycles" true 2> /dev/null
  then
    if ! perf record -o "${perfdata}" -e "cycles:P" true 2> /dev/null
    then
      echo "precise_max attribute [Failed cycles:P event]"
      err=1
      return
    fi
  else
    echo "precise_max attribute [Skipped no cycles:P event]"
    ((skipped+=1))
  fi
  # On s390 event instructions is not supported for perf record
  if perf record -o "${perfdata}" -e "instructions" true 2> /dev/null
  then
    # On AMD, cycles and instructions events are treated differently
    if ! perf record -o "${perfdata}" -e "instructions:P" true 2> /dev/null
    then
      echo "precise_max attribute [Failed instructions:P event]"
      err=1
      return
    fi
  else
    echo "precise_max attribute [Skipped no instructions:P event]"
    ((skipped+=1))
  fi
  if [ $skipped -eq 2 ]
  then
    echo "precise_max attribute [Skipped no hardware events]"
  else
    echo "precise_max attribute test [Success]"
  fi
}

test_callgraph() {
  echo "Callgraph test"

  case $(uname -m)
  in s390x)
       cmd_flags="--call-graph dwarf -e cpu-clock";;
     *)
       cmd_flags="-g";;
  esac

  if ! perf record -o "${perfdata}" $cmd_flags perf test -w brstack
  then
    echo "Callgraph test [Failed missing output]"
    err=1
    return
  fi

  if ! perf report -i "${perfdata}" 2>&1 | grep "${testsym2}"
  then
    echo "Callgraph test [Failed missing symbol]"
    err=1
    return
  fi

  echo "Callgraph test [Success]"
}

test_ratio_to_prev() {
  echo "ratio-to-prev test"
  if ! perf record -o /dev/null -e "{instructions, cycles/period=100000,ratio-to-prev=0.5/}" \
     true 2> /dev/null
  then
    echo "ratio-to-prev [Skipped not supported]"
    return
  fi
  if ! perf record -o /dev/null -e "instructions, cycles/period=100000,ratio-to-prev=0.5/" \
     true |& grep -q 'Invalid use of ratio-to-prev term without preceding element in group'
  then
    echo "ratio-to-prev test [Failed elements must be in same group]"
    err=1
    return
  fi
  if ! perf record -o /dev/null -e "{instructions,dummy,cycles/period=100000,ratio-to-prev=0.5/}" \
     true |& grep -q 'must have same PMU'
  then
    echo "ratio-to-prev test [Failed elements must have same PMU]"
    err=1
    return
  fi
  if ! perf record -o /dev/null -e "{instructions,cycles/ratio-to-prev=0.5/}" \
     true |& grep -q 'Event period term or count (-c) must be set when using ratio-to-prev term.'
  then
    echo "ratio-to-prev test [Failed period must be set]"
    err=1
    return
  fi
  if ! perf record -o /dev/null -e "{cycles/ratio-to-prev=0.5/}" \
     true |& grep -q 'Invalid use of ratio-to-prev term without preceding element in group'
  then
    echo "ratio-to-prev test [Failed need 2+ events]"
    err=1
    return
  fi
  echo "Basic ratio-to-prev record test [Success]"
}

# raise the limit of file descriptors to minimum
if [[ $default_fd_limit -lt $min_fd_limit ]]; then
       ulimit -Sn $min_fd_limit
fi

test_per_thread
test_register_capture
test_egpr_register_capture
test_simd_register_capture
test_system_wide
test_workload
test_branch_counter
test_cgroup
test_uid
test_leader_sampling
test_topdown_leader_sampling
test_precise_max
test_callgraph
test_ratio_to_prev

# restore the default value
ulimit -Sn $default_fd_limit

cleanup
exit $err
