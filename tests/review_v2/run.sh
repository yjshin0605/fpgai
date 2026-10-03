#!/bin/sh
set -eu
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
for name in tb_review tb_reset tb_axi_diag; do
    iverilog -g2012 -s "$name" -o "$out/$name.vvp" "tests/review_v2/$name.v" \
        spec_feat_axi.v spec_feat.v cordic_mag.v traj_frame.v traj_rec.v env_detect.v env_hist.v
    vvp "$out/$name.vvp"
done
