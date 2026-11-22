import assert from "node:assert/strict";
import test from "node:test";
import {Playback} from "../bench/viewer/episode.js";

function replay(moves, replans = []) {
    return {frames: moves.map(moved => ({moved})), replans: new Set(replans)};
}

test("search stays on the same move and a completed side keeps its final frame", () => {
    const left = replay([0, 0, 1, 1, 1], [3]);
    const right = replay([0, 0, 0, 1, 2, 2], [5]);
    const sequence = new Playback([left, right]);
    sequence.finished.fill(true);
    sequence.extend();
    assert.deepEqual(sequence.positions, [[0,0], [1,1], [1,2], [2,3], [3,3], [4,3], [4,4], [4,5]]);
    assert.deepEqual(sequence.replans, [4,7]);

    // The same records arriving on different connections must yield the same slider positions.
    const delayed = replay([0]);
    const streaming = new Playback([delayed, right]);
    streaming.finished[1] = true;
    streaming.extend();
    assert.equal(streaming.positions.length, 1);
    delayed.replans = left.replans;
    for (const frame of left.frames.slice(1)) {
        delayed.frames.push(frame);
        streaming.extend();
    }
    streaming.finished[0] = true;
    streaming.extend();
    assert.deepEqual(streaming.positions, sequence.positions);
    assert.deepEqual(streaming.replans, sequence.replans);
});

test("single mode streams all frames immediately", () => {
    const sequence = new Playback([replay([0,0,1,1], [3])]);
    sequence.extend();
    assert.deepEqual(sequence.positions, [[0],[1],[2],[3]]);
    assert.deepEqual(sequence.replans, [3]);
});
