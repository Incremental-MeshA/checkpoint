import assert from "node:assert/strict";
import test from "node:test";
import {Playback, Replay} from "../bench/viewer/episode.js";

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

test("single expansions merge across unequal batches and rewind with exact counters", () => {
    const left = replay([0,0,1]), right = replay([0,0,0,1]);
    left.frames[1].steps = 4;
    right.frames[2].steps = 2;
    const sequence = new Playback([left,right]);
    sequence.finished.fill(true);
    sequence.extend();
    assert.deepEqual(Array.from({length:sequence.last+1}, (_, i)=>sequence.indicesAt(i)),
        [[0,0],[1,1],[2,2],[3,3],[4,3],[5,4]]);

    const segment = {count:5, expanded:3, checkedCells:9, cellChecks:60,
                     stateCounts:[1,2,2,4,5,20,7,37,9,60]};
    const r = Object.assign(Object.create(Replay.prototype), {ends:[0,1,4], frames:[{}, {}, segment]});
    for (const [index,expansions,unique,reads] of [[4,3,9,60],[2,1,5,20],[3,2,7,37]]) {
        const f = r.frameAt(index);
        assert.deepEqual([f.expanded,f.checkedCells,f.cellChecks],[expansions,unique,reads]);
    }
});

test("blocking an interior trace cell marks the cached pointer until BuildPath redirects it", () => {
    const origin = [0,0,0];
    const r = Object.assign(Object.create(Replay.prototype), {
        spec:{logical:true,start:origin,graph:"lattice"}, moves:[], frames:[{}], ends:[0],
        replans:new Set(), known:{width:4,edits:[]}, latestKnown:new Uint8Array(16),
        controls:{1:{dx:2,dy:0,end:0,trace:[[0,0],[1,0],[2,0]]},
                  2:{dx:0,dy:1,end:1,trace:[[0,0],[0,1]]}}, nextByPose:new Map()
    });
    r.actual=r.known;
    const result={kind:"result",outcome:"found",trace:[],cell_frames:[],reuse_from_edge:null,
                  walkthrough:{started:true,opened:[0],raised:[0]},search_expanded:0};
    r.record({...result,route:[1,1]});
    const firstWrite=r.frames.at(-2);
    r.record({kind:"request",event:1,block:[1],clear:[]});
    const blockedFrame=r.frames.at(-1);
    assert.deepEqual(blockedFrame.blockedNext.map(s=>s.id),[1]);
    r.record({...result,route:[2]});
    const writeFrame=r.frames.at(-2);
    assert.deepEqual(firstWrite.storedNext,[]);
    assert.deepEqual(writeFrame.storedNext.map(s=>s.pose),[[2,0,0]]);
    assert.deepEqual(writeFrame.previousNext.map(s=>s.id),[1]);
    assert.deepEqual(writeFrame.pointers.map(s=>s.id),[2]);
    assert.equal(r.nextByPose.get(origin.join(",")).id,2);
    assert.equal(blockedFrame.blockedNext[0].id,1);
});
