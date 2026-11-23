// Map edits are reversible because each batch changes a cell at most once.
export class MapLayer {
    constructor(base, width) {
        this.width = width;
        this.cells = base.slice();
        this.edits = [];
        this.position = 0;
        this.canvas = document.createElement("canvas");
        this.canvas.width = width;
        this.canvas.height = base.length / width;
        this.ctx = this.canvas.getContext("2d");
        this.ctx.fillStyle = "#ffffff";
        this.ctx.fillRect(0, 0, width, this.canvas.height);
        this.ctx.fillStyle = "#b7d7b3";
        for (let i = 0; i < base.length; i++) {
            if (base[i]) this.ctx.fillRect(i % width, Math.floor(i / width), 1, 1);
        }
    }
    paint(cells, blocked) {
        this.ctx.fillStyle = blocked ? "#b7d7b3" : "#ffffff";
        for (const i of cells) {
            this.cells[i] = blocked;
            this.ctx.fillRect(i % this.width, Math.floor(i / this.width), 1, 1);
        }
    }
    seek(position) {
        while (this.position !== position) {
            const forward = this.position < position;
            const edit = this.edits[forward ? this.position++ : --this.position];
            this.paint(edit.block, Number(forward));
            this.paint(edit.clear, Number(!forward));
        }
    }
}

export const phases = ["Search", "Heuristic repair", "Update seeds"];
export const outcome = {reached: "Goal reached", no_path: "No path found"};

// Find the stored segment containing a virtual playback frame.
function segmentAt(ends, frame) {
    let lo = 0, hi = ends.length - 1;
    while (lo < hi) {
        const mid = Math.floor((lo + hi) / 2);
        if (ends[mid] < frame) lo = mid + 1;
        else hi = mid;
    }
    return lo;
}

export class Replay {
    constructor(base, width, spec, controls, animateEdges=false) {
        this.animateEdges = animateEdges;
        this.spec = spec;
        this.controls = controls;
        this.known = new MapLayer(base, width);
        this.actual = spec.scenario === "limited_visibility" ? new MapLayer(base, width) : this.known;
        if (spec.logical) {
            this.nextByPose = new Map();
            this.latestKnown = base.slice();
        }
        this.moves = [];
        this.replans = new Set();
        this.ends = [0];
        this.frames = [{phase: "Ready to run", moved: 0, known: 0, actual: 0, event: -1,
                        search: null, count: 0, expanded: 0, checkedCells: 0, cellChecks: 0, plan: null, offset: 0, edit: null}];
    }
    add(fields) {
        this.frames.push({...this.frames[this.frames.length - 1], edit: null, candidate: null,
                          learned: null, focus: null, pointers: null, blockedNext: null, storedNext: null, previousNext: null, steps: 1, stateCounts: null, ...fields});
        this.ends.push(this.ends[this.ends.length - 1] + (fields.steps || 1));
    }
    pose(moved) {
        return moved ? this.moves[moved - 1].pose : this.spec.start;
    }
    record(r) {
        switch (r.kind) {
        case "world":
            this.actual.edits.push(r);
            if (r.manual) break;
            this.add({phase: "World update", actual: this.actual.edits.length, edit: r});
            break;
        case "request": {
            const blockedNext = this.spec.logical ? this.applyPointerEdits(r) : null;
            this.known.edits.push(r);
            if (r.event) this.replans.add(this.frames.length);
            this.add({phase: r.event ? "Replanning" : "Initial planning request", event: r.event,
                      known: this.known.edits.length, actual: this.actual === this.known ?
                          this.known.edits.length : this.actual.edits.length,
                      edit: r, blockedNext, search: null, count: 0, expanded: 0, checkedCells: 0, cellChecks: 0, cost: null, plan: null, offset: 0});
            break;
        }
        case "result": {
            if (!Array.isArray(r.trace) || !Array.isArray(r.cell_frames))
                throw Error("Rebuild the planner to enable cell check counts.");
            const trace = Uint32Array.from(r.trace);
            if (this.spec.state_steps && r.state_counts?.length !== 2 * trace.length)
                throw Error("Restart the viewer and rebuild the planner to enable single-state steps.");
            const stateCounts = this.spec.state_steps ? Float64Array.from(r.state_counts) : null;
            const samples = this.animateEdges ? r.edge_samples : [];
            if (!samples) throw Error("Rebuild the planner to enable edge animation.");
            let sampleIndex = 0;
            let expanded = 0, previousCount = 0;
            if (this.spec.logical) this.logicalSearch(r, trace);
            for (const [count, checkedCells, cellChecks] of this.spec.logical ? [] : r.cell_frames) {
                const kind = trace[count - 1] % 4;
                // Each batch is a single phase. Only search and repair expand states.
                let phase = phases[kind];
                if (!kind && this.spec.algorithm === "dstar_lite")
                    phase = r.event ? "Propagating updates" : "Searching backward from the goal";
                if (!kind && this.spec.algorithm === "astar") phase = "Searching from the current pose";
                while (sampleIndex < samples.length && samples[sampleIndex][0] <= count) {
                    const [event, edges] = samples[sampleIndex++];
                    for (const candidate of edges) {
                        const [, , , , id, clear, unique, reads] = candidate;
                        const motions = candidate[9];
                        let label = this.spec.graph === "mesh" ? "Mesh edge" : "Primitive " + id;
                        if (motions && motions.length)
                            label = (motions.length === 1 ? "Primitive " + motions[0] :
                                     "Shared prefix (" + motions.length + " primitives)") + " · edge";
                        this.add({phase: label + (clear ? " · clear" : " · blocked"), candidate,
                                  search: trace, count: event, checkedCells: unique, cellChecks: reads,
                                  expanded: expanded + (kind === 2 ? 0 : event - previousCount)});
                    }
                }
                const expansions = kind === 2 ? 0 : count - previousCount;
                expanded += expansions;
                previousCount = count;
                this.add({phase, search: trace, count, expanded, checkedCells, cellChecks,
                          steps: stateCounts && expansions ? expansions : 1, stateCounts});
            }
            const origin = this.pose(this.moves.length);
            const plan = [];
            let pose = origin, edge = 0;
            for (const id of r.route) {
                const p = this.controls[id];
                const edges = this.spec.graph === "mesh" ? p.trace.length - 1 : 1;
                const reuse = r.reuse_from_edge;
                const cut = reuse !== null && reuse < edge + edges ? Math.max(0, reuse - edge) : null;
                plan.push({pose, id, cut});
                pose = [pose[0] + p.dx, pose[1] + p.dy, p.end];
                edge += edges;
            }
            if (this.spec.logical && r.outcome === "found") {
                const prefix = plan.slice(0, r.reuse_from_edge ?? plan.length);
                if (prefix.length) {
                    const previousNext = [];
                    for (const step of prefix) {
                        const key = step.pose.join(",");
                        const old = this.nextByPose.get(key);
                        if (old && old.id !== step.id) previousNext.push(old);
                        this.nextByPose.set(key, step);
                    }
                    const written = new Set(prefix);
                    const storedNext = Array.from(this.nextByPose.values()).filter(step => !written.has(step));
                    this.add({phase: "BuildPath · Write next pointers (" + prefix.length + " writes, " +
                              previousNext.length + " redirected)", pointers: prefix, storedNext, previousNext});
                }
                if (r.reuse_from_edge != null)
                    this.add({phase: "Follow the verified next chain · Reuse " + (plan.length-prefix.length) + " edges to the goal", plan});
            }
            this.add({phase: r.outcome === "found" ? "Route found" : outcome.no_path,
                      plan, offset: 0, expanded: r.expanded, checkedCells: r.checked_cells, cellChecks: r.cell_checks, cost: r.cost});
            break;
        }
        case "move": {
            this.moves.push({origin: this.pose(this.moves.length), id: r.primitive, pose: r.pose});
            const previous = this.frames[this.frames.length - 1];
            this.add({phase: "Moving", moved: this.moves.length, offset: previous.offset + 1});
            break;
        }
        case "summary":
            this.add({phase: outcome[r.status] || r.status});
            break;
        case "error":
            throw Error(r.error);
        }
    }
    applyPointerEdits(edit) {
        const blocked = new Set(edit.block), affected = [];
        if (blocked.size) {
            for (const step of this.nextByPose.values()) {
                const [x,y] = step.pose;
                let newlyBlocked = false, wasClear = true;
                for (const [dx,dy] of this.controls[step.id].trace) {
                    const cell = (y+dy)*this.known.width+x+dx;
                    if (this.latestKnown[cell]) { wasClear = false; break; }
                    if (blocked.has(cell)) newlyBlocked = true;
                }
                if (wasClear && newlyBlocked) affected.push(step);
            }
        }
        for (const cell of edit.block) this.latestKnown[cell] = 1;
        for (const cell of edit.clear) this.latestKnown[cell] = 0;
        return affected;
    }
    logicalSearch(r, trace) {
        const w = r.walkthrough;
        if (!w) throw Error("Rebuild the planner to enable MPGAA* logical steps.");
        const edit = this.known.edits.at(-1);
        if (edit && (edit.block.length || edit.clear.length)) {
            const detail = edit.clear.length
                ? w.opened[0] + " newly traversable edges from poses with learned h"
                : "Skipped (no cells cleared)";
            this.add({phase: "Check watched edges · " + detail, edit, focus: w.opened});
        }
        // The native recorder splits its batches at every phase boundary.
        const totals = (repair) => {
            const row = r.cell_frames.filter(([end]) => (trace[end-1] % 4 !== 0) === repair).at(-1);
            if (!row) return {};
            const [count, checkedCells, cellChecks] = row;
            return {search: trace, count, checkedCells, cellChecks};
        };
        const describe = ([count,x,y,d,,before,after,oldSteps,newSteps]) => count ? " · Example (" +
            [x,y,d].join(", ") + "): " + before.toFixed(2) + " (" + oldSteps + " steps) → " + after.toFixed(2) + " (" + newSteps + " steps)" : "";
        if (edit?.clear.length)
            this.add({phase: "Repair h · Propagate decreases to predecessors. " + w.lowered[0] +
                      " updates" + describe(w.lowered), ...totals(true), expanded:r.repair_expanded, focus:w.lowered});
        if (!w.started) {
            this.add({phase: "Endpoint check · An isolated endpoint prevents a search"});
            return;
        }
        this.add({phase: "Forward search · A*", ...totals(false), expanded:r.expanded});
        if (r.outcome === "found" && r.search_expanded)
            this.add({phase: "Learn h · h(u) = f(Goal) - g(u)", learned:Uint32Array.from(w.learned_poses)});
    }
    frameAt(index) {
        const segment = segmentAt(this.ends, index);
        const frame = this.frames[segment], remaining = this.ends[segment] - index;
        if (!remaining) return frame;
        const count = frame.count - remaining;
        return {...frame, count, expanded: frame.expanded - remaining,
                checkedCells: frame.stateCounts[2*count-2], cellChecks: frame.stateCounts[2*count-1]};
    }
    seek(index) {
        const frame = this.frameAt(index);
        this.known.seek(frame.known);
        if (this.known !== this.actual) this.actual.seek(frame.actual);
        return frame;
    }
}

// Advance search frames within the same move before allowing either robot to move again.
// Waiting for the next record on each unfinished side makes seeking independent of network timing.
export class Playback {
    constructor(replays) {
        this.replays = replays;
        this.finished = replays.map(() => false);
        this.positions = [replays.map(() => 0)];
        this.ends = [0];
        this.nextFrame = replays.map(() => 1);
        this.used = replays.map(() => 0);
        this.replans = [];
    }
    get last() { return this.ends[this.ends.length - 1]; }
    indicesAt(frame) {
        const segment = segmentAt(this.ends, frame);
        const position = this.positions[segment], remaining = this.ends[segment] - frame;
        if (!remaining) return position;
        return position.map((index,i) => index === this.positions[segment-1][i] ? index : index-remaining);
    }
    extend() {
        let previous = this.positions[this.positions.length - 1];
        while (true) {
            const next = this.replays.map((r, i) => r.frames[this.nextFrame[i]]);
            if (next.some((f, i) => !f && !this.finished[i])) return;
            const move = Math.min(...next.map(f => f ? f.moved : Infinity));
            if (!Number.isFinite(move)) return;
            // Merge whole runs of single-state steps without allocating a JS object per expansion.
            const length = Math.min(...next.map((f,i) => f?.moved === move ? (f.steps || 1)-this.used[i] : Infinity));
            let replan = false;
            const position = previous.map((index, i) => {
                if (next[i]?.moved !== move) return index;
                if (!this.used[i] && this.replays[i].replans.has(this.nextFrame[i])) replan = true;
                this.used[i] += length;
                if (this.used[i] === (next[i].steps || 1)) {
                    this.used[i] = 0;
                    this.nextFrame[i]++;
                }
                return index + length;
            });
            if (replan) this.replans.push(this.last + 1);
            this.ends.push(this.last + length);
            this.positions.push(position);
            previous = position;
        }
    }
}
