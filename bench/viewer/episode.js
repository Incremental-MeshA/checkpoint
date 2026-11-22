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

export class Replay {
    constructor(base, width, spec, controls, animateEdges=false) {
        this.animateEdges = animateEdges;
        this.spec = spec;
        this.controls = controls;
        this.known = new MapLayer(base, width);
        this.actual = spec.scenario === "limited_visibility" ? new MapLayer(base, width) : this.known;
        this.moves = [];
        this.replans = new Set();
        this.frames = [{phase: "Ready to run", moved: 0, known: 0, actual: 0, event: -1,
                        search: null, count: 0, expanded: 0, checkedCells: 0, cellChecks: 0, plan: null, offset: 0, edit: null}];
    }
    add(fields) {
        this.frames.push({...this.frames[this.frames.length - 1], edit: null, candidate: null, ...fields});
    }
    pose(moved) {
        return moved ? this.moves[moved - 1].pose : this.spec.start;
    }
    record(r) {
        switch (r.kind) {
        case "world":
            this.actual.edits.push(r);
            this.add({phase: "World update", actual: this.actual.edits.length, edit: r});
            break;
        case "request": {
            this.known.edits.push(r);
            const current = this.frames[this.frames.length - 1];
            if (r.event) this.replans.add(this.frames.length);
            this.add({phase: r.event ? "Replanning" : "Initial planning request", event: r.event,
                      known: this.known.edits.length, actual: this.actual === this.known ?
                          this.known.edits.length : current.actual,
                      edit: r, search: null, count: 0, expanded: 0, checkedCells: 0, cellChecks: 0, cost: null, plan: null, offset: 0});
            break;
        }
        case "result": {
            if (!Array.isArray(r.trace) || !Array.isArray(r.cell_frames))
                throw Error("Rebuild the planner to enable cell check counts.");
            const trace = Uint32Array.from(r.trace);
            const samples = this.animateEdges ? r.edge_samples : [];
            if (!samples) throw Error("Rebuild the planner to enable edge animation.");
            let sampleIndex = 0;
            let expanded = 0, previousCount = 0;
            for (const [count, checkedCells, cellChecks] of r.cell_frames) {
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
                if (kind !== 2) expanded += count - previousCount;
                previousCount = count;
                this.add({phase, search: trace, count, expanded, checkedCells, cellChecks});
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
    seek(index) {
        const frame = this.frames[index];
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
        this.replans = [];
    }
    extend() {
        let previous = this.positions[this.positions.length - 1];
        while (true) {
            const next = this.replays.map((r, i) => r.frames[previous[i] + 1]);
            if (next.some((f, i) => !f && !this.finished[i])) return;
            const move = Math.min(...next.map(f => f ? f.moved : Infinity));
            if (!Number.isFinite(move)) return;
            const position = previous.map((index, i) =>
                next[i] && next[i].moved === move ? index + 1 : index);
            if (position.some((index, i) => index !== previous[i] && this.replays[i].replans.has(index)))
                this.replans.push(this.positions.length);
            this.positions.push(position);
            previous = position;
        }
    }
}
