import {Playback, Replay} from "./episode.js";
import {MapView} from "./map.js";

const $ = id => document.getElementById(id);
const names = {astar:"A*", dstar_lite:"D* Lite", mpgaa:"MPGAA*",
    dstar_lite_chain:"D* Lite (chains)", mpgaa_chain:"MPGAA* (chains)",
    navigation_replanning:"Fully observed changes", limited_visibility:"Limited visibility"};
const panels = [
    {slot:"left", canvas:$("mapView"), legend:$("legend"), title:$("titleLeft"), phase:$("phase"),
     stats:$("stats"), algorithm:$("algorithm"), graph:$("graph")},
    {slot:"right", canvas:$("mapRight"), legend:$("legendRight"), title:$("titleRight"), phase:$("phaseRight"),
     stats:$("statsRight"), algorithm:$("algorithm2"), graph:$("graph2")}
];
let catalog, controls, playback, grid, width, tasks, start, goal;
let comparing = false;
let frame = 0, playing = false, active = null, mapVersion = 0, placement = null;
let scheduled = false, timer = 0, loading = false;

function options(id, values, label=value=>names[value] || value) {
    $(id).replaceChildren(...values.map(value=>new Option(label(value),value)));
}
function settings(panel) {
    return {map:$("map").value, task:$("task").value, start:[...start], goal:[...goal],
        scenario:$("scenario").value, algorithm:panel.algorithm.value, graph:panel.graph.value,
        percent:Number($("percent").value), radius:Number($("radius").value), seed:$("seed").value,
        state_steps:$("detail").value==="state"};
}
function update() {
    if (scheduled) return;
    scheduled=true;
    requestAnimationFrame(()=>{
        scheduled=false;
        if (!playback) return;
        const last=playback.last;
        const positions=playback.indicesAt(frame);
        playback.replays.forEach((replay, i) => {
            const panel=panels[i], f=replay.seek(positions[i]);
            panel.view.draw(replay,f,$("world").checked);
            panel.phase.textContent=active && !last ? "Planning route…" : f.phase;
            const cost=f.cost == null ? "—" : f.cost.toLocaleString("en",{maximumFractionDigits:2});
            panel.stats.textContent=f.event < 0 ? "" : "Move "+f.moved+" · Request "+(f.event+1)+
                " · Cell checks "+f.cellChecks.toLocaleString("en")+
                " · State expansions "+f.expanded.toLocaleString("en")+
                (comparing ? " · Path cost "+cost : "");
            panel.stats.title=f.event < 0 ? "" : "Distinct cells checked this request: "+f.checkedCells.toLocaleString("en");
        });
        $("timeline").max=last; $("timeline").value=frame; $("timeline").disabled=!last;
        $("prev").disabled=!frame; $("next").disabled=frame===last;
        $("play").disabled=!last && !active;
        $("play").textContent=playing ? "Pause" : "Play";
        $("replan").disabled=!playback.replans.some(i=>i>frame);
    });
}
function pause() {
    playing=false;clearTimeout(timer);update();
}
function play() {
    clearTimeout(timer);
    playing=true;
    function tick() {
        if (!playing) return;
        if (frame<playback.last) frame++;
        else if (!active) return pause();
        update();
        timer=setTimeout(tick,160/parseFloat($("speed").value));
    }
    tick();
}
function post(path,data,signal) {
    return fetch(path,{method:"POST",headers:{"Content-Type":"application/json"},
        body:JSON.stringify(data),signal});
}
function cancel() {
    pause();
    if (!active) return;
    const job=active; active=null;
    job.controller.abort();
    for (const slot of job.slots) post("/api/cancel",{key:job.key,slot}).catch(()=>{});
}
function reset() {
    cancel();
    if (loading || !grid) return;
    for (const panel of panels) {
        const allowed = catalog.variants.filter(([algo]) => algo === panel.algorithm.value)
                                       .map(([, graph]) => graph);
        for (const option of panel.graph.options) option.disabled = !allowed.includes(option.value);
        if (!allowed.includes(panel.graph.value)) panel.graph.value = allowed[0];
    }
    frame=0;
    const visible=comparing ? panels : panels.slice(0,1);
    playback=new Playback(visible.map(panel=>{
        const spec=settings(panel);
        panel.title.textContent=names[spec.algorithm]+" · "+spec.graph;
        panel.view.selected=null;
        panel.view.trace=null;
        return new Replay(grid,width,spec,controls,$("detail").value==="edges");
    }));
    $("error").textContent="";
    update();
}
function setPlacement(value) {
    placement=value;
    $("hint").hidden=!value;
    $("hint").textContent=value==="start" ? "Click a free cell to set the start" : "Click a free cell to set the goal";
    $("setStart").setAttribute("aria-pressed",value==="start");
    $("setGoal").setAttribute("aria-pressed",value==="goal");
}
function focus(whole=false) {
    if (!playback) return;
    const view=panels[0].view;
    view.focus(start,goal,playback.replays[0].known,whole);
    syncCamera(view);
}
function syncCamera(source) {
    if (comparing) {
        for (const {view} of panels) {
            view.scale=source.scale;
            view.center=[...source.center];
        }
    }
    update();
}
function setMode(two) {
    if (comparing===two) return;
    comparing=two;
    $("maps").classList.toggle("two",two);
    $("single").setAttribute("aria-pressed",!two);
    $("compare").setAttribute("aria-pressed",two);
    $("secondPlanner").hidden=$("rightPanel").hidden=$("titleLeft").hidden=!two;
    $("firstPlanner").textContent=two ? "Left planner" : "Planner";
    for (const {view} of panels) view.resize();
    reset();focus();
}
function taskChanged() {
    const manual=$("task").value==="manual";
    $("manual").hidden=!manual;
    setPlacement(manual ? "start" : null);
    if (!manual) {
        const t=tasks.find(item=>item.id===$("task").value);
        start=[...t.start];goal=[...t.goal];
    }
    $("startH").value=start[2];$("goalH").value=goal[2];
    reset();focus();
}
async function mapChanged() {
    const version=++mapVersion;
    cancel();loading=true;grid=null;playback=null;
    $("run").disabled=true;$("phase").textContent="Loading map…";
    for (const id of ["timeline","prev","next","play","replan","task"]) $(id).disabled=true;
    $("manual").hidden=true;
    setPlacement(null);
    const name=$("map").value;
    const response=await fetch("/api/maps/"+encodeURIComponent(name));
    if (!response.ok) throw Error("Could not load the map");
    const data=new Uint8Array(await response.arrayBuffer());
    if (version!==mapVersion) return;
    width=Number(response.headers.get("X-Map-Width"));grid=data;
    tasks=catalog.tasks.filter(t=>t.map===name);
    options("task",tasks.map(t=>t.id).concat("manual"),id=>{
        if (id==="manual") return "Choose on map";
        const t=tasks.find(t=>t.id===id);
        return "Task "+t.line+" · "+t.band;
    });
    $("mapTitle").textContent=name;
    $("dimensions").textContent=width+" × "+grid.length/width+" cells";
    loading=false;
    taskChanged();$("run").disabled=false;$("task").disabled=false;
}
function scenarioChanged() {
    const cfg=catalog.scenarios[$("scenario").value];
    $("percent").value=cfg.change_percent;$("seed").value=cfg.seed;
    const visible=Boolean(cfg.radii);
    $("radiusField").hidden=!visible;$("worldField").hidden=!visible;$("world").checked=false;
    options("radius",cfg.radii || []);
    if (visible) $("radius").value=cfg.radius;
    reset();
}
async function run(event) {
    event.preventDefault();
    if (loading || !grid) return;
    setPlacement(null);reset();
    const job={key:crypto.randomUUID(),controller:new AbortController(),
               slots:playback.replays.map((_,i)=>panels[i].slot)};
    active=job;
    if (!playback.replays[0].spec.state_steps) play();
    await Promise.all(playback.replays.map((replay,i)=>streamEpisode(job,replay,i)));
    if (active===job) active=null;
    update();
}
async function streamEpisode(job,replay,i) {
    const slot=job.slots[i];
    try {
        const response=await post("/api/run",{...replay.spec,key:job.key,slot},job.controller.signal);
        if (!response.ok) throw Error((await response.json()).error || "Could not start the episode");
        const reader=response.body.pipeThrough(new TextDecoderStream()).getReader();
        let parts=[], finished=false;
        while (active===job) {
            const {value,done}=await reader.read();
            if (active!==job || done) break;
            const lines=value.split("\n");
            for (let i=0; i<lines.length-1; i++) {
                parts.push(lines[i]);
                const record=JSON.parse(parts.join(""));
                parts=[];
                replay.record(record);
                if (record.kind==="summary") finished=true;
            }
            parts.push(lines[lines.length-1]);
            playback.extend();
            update();
        }
        if (active===job && !finished) throw Error("Planning stopped before the episode finished");
    } catch (error) {
        if (active===job) {
            replay.add({phase:"Planning error"});
            const label=comparing ? panels[i].title.textContent+" — " : "";
            $("error").textContent=[ $("error").textContent, label+error.message ].filter(Boolean).join(" · ");
            post("/api/cancel",{key:job.key,slot}).catch(()=>{});
        }
    } finally {
        if (active===job) {
            playback.finished[i]=true;
            playback.extend();
            update();
        }
    }
}

function bind() {
    $("single").onclick=()=>setMode(false);
    $("compare").onclick=()=>setMode(true);
    $("map").onchange=()=>mapChanged().catch(showError);
    $("task").onchange=taskChanged;$("scenario").onchange=scenarioChanged;
    for (const id of ["radius","percent","seed","graph","algorithm","graph2","algorithm2","detail"]) $(id).onchange=reset;
    $("setStart").onclick=()=>setPlacement("start");$("setGoal").onclick=()=>setPlacement("goal");
    $("startH").onchange=()=>{start[2]=Number($("startH").value);reset();};
    $("goalH").onchange=()=>{goal[2]=Number($("goalH").value);reset();};
    $("settings").onsubmit=run;
    $("world").onchange=update;
    const view=panels[0].view;
    $("in").onclick=()=>{view.zoom(1.6);syncCamera(view);};
    $("out").onclick=()=>{view.zoom(1/1.6);syncCamera(view);};
    $("fit").onclick=()=>focus(true);$("focus").onclick=()=>focus();
    $("timeline").oninput=()=>{pause();frame=Number($("timeline").value);update();};
    $("prev").onclick=()=>{pause();frame=Math.max(0,frame-1);update();};
    $("next").onclick=()=>{pause();frame=Math.min(playback.last,frame+1);update();};
    $("replan").onclick=()=>{pause();frame=playback.replans.find(i=>i>frame) ?? frame;update();};
    $("play").onclick=()=>{
        if (playing) return pause();
        if (!active && frame===playback.last) frame=0;
        play();
    };
    panels.forEach(bindCanvas);
    window.addEventListener("pagehide",()=>{
        if (!active) return;
        for (const slot of active.slots) {
            const data=JSON.stringify({key:active.key,slot});
            navigator.sendBeacon("/api/cancel",new Blob([data],{type:"application/json"}));
        }
    });
}
function bindCanvas({canvas,view},i) {
    let drag=null;
    canvas.onwheel=e=>{
        e.preventDefault();
        view.zoom(e.deltaY<0 ? 1.22 : 1/1.22,e.offsetX,e.offsetY);syncCamera(view);
    };
    canvas.onpointerdown=e=>{
        drag={x:e.clientX,y:e.clientY,center:[...view.center],moved:false};
        canvas.setPointerCapture(e.pointerId);
    };
    canvas.onpointermove=e=>{
        if (!drag) return;
        const dx=e.clientX-drag.x, dy=e.clientY-drag.y;
        if (Math.hypot(dx,dy)>3) drag.moved=true;
        if (drag.moved) {
            view.center=[drag.center[0]-dx/view.scale,drag.center[1]-dy/view.scale];syncCamera(view);
        }
    };
    canvas.onpointercancel=()=>{drag=null;};
    canvas.onpointerup=e=>{
        const click=drag && !drag.moved;
        drag=null;
        if (!click || !playback) return;
        const [x,y]=view.point(e.offsetX,e.offsetY);
        if (!placement) {
            view.pick(playback.replays[i].frameAt(playback.indicesAt(frame)[i]),x,y);
            syncCamera(view);return;
        }
        const cx=Math.floor(x),cy=Math.floor(y);
        if (cx<0 || cx>=width || cy<0 || cy>=grid.length/width || grid[cy*width+cx]) {
            $("error").textContent="Choose a free cell on the map.";return;
        }
        const pose=placement==="start" ? start : goal;
        pose[0]=cx;pose[1]=cy;
        setPlacement(placement==="start" ? "goal" : null);
        reset();
    };
    new ResizeObserver(()=>{view.resize();update();}).observe(canvas);
}
function showError(error) {
    $("error").textContent=error.message;$("phase").textContent="Loading error";
}
async function init() {
    [catalog,controls]=await Promise.all(["catalog","primitives"].map(async name=>{
        const response=await fetch("/api/"+name);
        if (!response.ok) throw Error("Could not load the episode data");
        return response.json();
    }));
    for (const panel of panels) {
        panel.view=new MapView(panel.canvas,panel.legend,controls);
        panel.view.resize();
        options(panel.algorithm.id,catalog.algorithms);
        options(panel.graph.id,catalog.graphs);
    }
    $("algorithm2").value="dstar_lite";
    options("map",catalog.maps);options("scenario",Object.keys(catalog.scenarios));
    for (const id of ["startH","goalH"]) options(id,[...Array(16).keys()],h=>h+" · "+Math.round(panels[0].view.angles[h]*180/Math.PI+360)%360+"°");
    scenarioChanged();await mapChanged();bind();
}
init().catch(showError);
