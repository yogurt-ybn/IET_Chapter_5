#!/usr/bin/env python3
"""Run the frozen common matrix sequentially on one isolated simulator; preserve failures."""

import argparse
import copy
import json
import os
from pathlib import Path

from iet_ch5.logging import config_hash, dump, revision
from iet_ch5.runner import run


def matrix():
    methods = ["pid","lqr","geometric","dob","mpc"]
    for method in methods:
        for repeat in range(2):
            yield method,"hover_transfer",0,repeat,True
        for task in ["circle","disturbance","mismatch"]:
            yield method,task,0,0,True
        for seed in range(5):
            yield method,"noise_delay",seed,0,True
    yield "dob","disturbance",0,0,False


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--local-config",type=Path,required=True)
    p.add_argument("--exclusive-simulator",action="store_true",required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--resume",action="store_true",help="Verify frozen manifest before reusing finished runs")
    args = p.parse_args()
    local = json.loads(args.local_config.read_text())
    controllers = json.loads(Path("configs/controllers.json").read_text())
    tasks = json.loads(Path("configs/tasks.json").read_text())
    frozen = {"controllers":controllers,"tasks":tasks,"matrix":list(matrix()),"software":revision(),
              "thread_settings":{k:os.environ.get(k) for k in ["OPENBLAS_NUM_THREADS","OMP_NUM_THREADS"]}}
    manifest = {"freeze":frozen,"sha256":config_hash(frozen)}
    args.output.mkdir(parents=True,exist_ok=args.resume)
    path = args.output/"manifest.json"
    if args.resume and path.exists():
        if json.loads(path.read_text()) != json.loads(dump(manifest)):
            raise RuntimeError("Resume rejected: source/configuration/matrix differs from frozen manifest")
    else:
        path.write_text(dump(manifest)+"\n")
    results = []
    for index,(controller,task,seed,repeat,enabled) in enumerate(matrix()):
        name = f"{index:02d}-{controller}-{task}-s{seed}-r{repeat}"+("-off" if not enabled else "")
        out = args.output/name
        if args.resume and (out/"result.json").exists():
            result = json.loads((out/"result.json").read_text())
        else:
            cfg = copy.deepcopy(controllers)
            if controller == "dob":
                cfg["dob"]["enabled"] = enabled
            value = run(local,cfg,tasks,controller,task,out,seed=seed)
            result = json.loads(dump(value))
        result.update(task=task,seed=seed,repeat=repeat,observer_enabled=enabled)
        results.append(result)
        (args.output/"summary.json").write_text(dump(results)+"\n")
        print(dump({"completed":index+1,"total":51,"run":name,"status":result["status"],
                    "rms_m":result["metrics"].get("position_rms_m"),"failure":result["failure"]}),flush=True)
