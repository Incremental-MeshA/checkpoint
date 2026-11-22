const ink = {
    "Edge check": "#5543a0", "Blocked edge": "#a33b2e",
    "Primitive trajectory": "#6355ad", "Possible primitives": "#9990bc",
    "Obstacles": "#b7d7b3", "Start": "#146339", "Goal": "#ac4c1c", "Robot": "#17251a",
    "Search": "#c6b8ee", "Mesh prefixes": "#a9bcf5", "Heuristic repair": "#e9a9d0",
    "Update seeds": "#edb751", "Planned path": "#334dc3", "Path traveled": "#443554",
    "Reused path": "#a53083", "Maneuvers": "#6d9a66", "Selected maneuver": "#3038a1",
    "Sensor radius": "#4b9085", "Newly blocked": "#c4744c", "Newly cleared": "#84bc96"
};
const traceLayers = ["Search", "Heuristic repair", "Update seeds"];

export class MapView {
    constructor(canvas, legend, controls) {
        this.canvas = canvas;
        this.ctx = canvas.getContext("2d");
        this.legend = legend;
        this.controls = controls;
        this.byHeading = Array.from({length:16}, () => []);
        this.angles = [];
        for (const p of Object.values(controls)) {
            this.byHeading[p.start].push(p);
            if (p.start === p.end) this.angles[p.start] = Math.atan2(p.dy, p.dx);
        }
        this.center = [0, 0];
        this.scale = 1;
    }
    resize() {
        const ratio = window.devicePixelRatio || 1;
        this.w = this.canvas.clientWidth;
        this.h = this.canvas.clientHeight;
        this.canvas.width = Math.round(this.w * ratio);
        this.canvas.height = Math.round(this.h * ratio);
    }
    focus(start, goal, layer, whole) {
        this.center = whole ? [layer.width/2, layer.canvas.height/2] :
            [(start[0]+goal[0]+1)/2, (start[1]+goal[1]+1)/2];
        const width = whole ? layer.width : Math.max(18, Math.abs(start[0]-goal[0])+14);
        const height = whole ? layer.canvas.height : Math.max(18, Math.abs(start[1]-goal[1])+14);
        this.scale = Math.min(48, (this.w-12)/width, (this.h-12)/height);
    }
    point(x, y) {
        return [(x-this.w/2)/this.scale+this.center[0], (y-this.h/2)/this.scale+this.center[1]];
    }
    zoom(factor, x=this.w/2, y=this.h/2) {
        const anchor = this.point(x, y);
        this.scale = Math.max(.2, Math.min(90, this.scale*factor));
        const next = this.point(x, y);
        this.center = this.center.map((v, i) => v+anchor[i]-next[i]);
    }
    marker(pose, name, flag=false) {
        const ctx = this.ctx;
        const size = Math.max(6 / this.scale, .5);
        const s = name === "Robot" ? size * .64 : size;
        ctx.save();
        ctx.translate(pose[0]+.5, pose[1]+.5);
        ctx.rotate(this.angles[pose[2]]);
        ctx.fillStyle = ink[name];
        ctx.strokeStyle = "white";
        ctx.lineWidth = 1.3/this.scale;
        ctx.beginPath();
        if (flag) {
            ctx.moveTo(-s,-s); ctx.lineTo(1.6*s,0); ctx.lineTo(-s,s); ctx.closePath();
        } else {
            ctx.moveTo(1.5*s,0); ctx.lineTo(-s,-s); ctx.lineTo(-.6*s,0); ctx.lineTo(-s,s); ctx.closePath();
        }
        ctx.fill(); ctx.stroke(); ctx.restore();
    }
    curve(pose, p, name, weight, from=0) {
        const ctx = this.ctx;
        ctx.strokeStyle = ink[name];
        ctx.lineWidth = weight/this.scale;
        ctx.beginPath();
        for (let i=from; i<p.curve.length; i++) {
            const [dx,dy] = p.curve[i];
            const x = pose[0]+.5+dx, y = pose[1]+.5+dy;
            if (i === from) ctx.moveTo(x,y);
            else ctx.lineTo(x,y);
        }
        ctx.stroke();
    }
    searchLayer(frame, layer, names) {
        if (!frame.search) return;
        if (this.trace !== frame.search || this.traceCount > frame.count) {
            this.overlay = document.createElement("canvas");
            this.overlay.width = layer.width;
            this.overlay.height = layer.canvas.height;
            this.trace = frame.search;
            this.traceCount = 0;
            this.traceNames = new Set();
        }
        const ctx = this.overlay.getContext("2d");
        while (this.traceCount < frame.count) {
            const code = this.trace[this.traceCount++];
            const cell = Math.floor(code/8);
            const kind = code % 4;
            const name = kind === 0 && (code & 4) ? "Mesh prefixes" : traceLayers[kind];
            ctx.fillStyle = ink[name];
            ctx.fillRect(cell % layer.width, Math.floor(cell/layer.width), 1, 1);
            this.traceNames.add(name);
        }
        this.ctx.globalAlpha = .77;
        this.ctx.drawImage(this.overlay, 0, 0);
        this.ctx.globalAlpha = 1;
        if (frame.count && frame.phase !== "Moving" && !frame.plan) {
            const cell = Math.floor(frame.search[frame.count - 1] / 8);
            this.ctx.strokeStyle = "#36525b";
            this.ctx.lineWidth = 1.4 / this.scale;
            this.ctx.strokeRect(cell % layer.width, Math.floor(cell/layer.width), 1, 1);
        }
        for (const name of this.traceNames) names.add(name);
    }
    inspect(replay, frame, nameSet) {
        const plan = frame.plan || [];
        let step = this.selected;
        if (!step || !plan.includes(step) || plan.indexOf(step) < frame.offset) step = plan[frame.offset];
        const pose = step ? step.pose : replay.pose(frame.moved);
        const layer = replay.known;
        for (const p of this.byHeading[pose[2]]) {
            const valid = p.trace.every(([dx,dy]) => {
                const x=pose[0]+dx, y=pose[1]+dy;
                return x>=0 && x<layer.width && y>=0 && y<layer.canvas.height && !layer.cells[y*layer.width+x];
            });
            if (!valid) continue;
            this.curve(pose,p,"Maneuvers",.85);
            const ctx = this.ctx;
            ctx.save();
            ctx.translate(pose[0]+.5+p.dx, pose[1]+.5+p.dy);
            ctx.rotate(this.angles[p.end]);
            const a = Math.min(.40,4/this.scale);
            ctx.beginPath(); ctx.moveTo(a,0); ctx.lineTo(-a,-a*.65); ctx.lineTo(-a,a*.65);
            ctx.fillStyle=ink["Maneuvers"]; ctx.fill(); ctx.restore();
            nameSet.add("Maneuvers");
        }
        if (step) {
            this.curve(pose,this.controls[step.id],"Selected maneuver",1.8);
            nameSet.add("Selected maneuver");
        }
    }
    pick(frame, x, y) {
        const plan = (frame.plan || []).slice(frame.offset);
        const distance = step => Math.hypot(step.pose[0]+.5-x,step.pose[1]+.5-y);
        let best = null;
        for (const step of plan) if (!best || distance(step)<distance(best)) best=step;
        if (best && distance(best) <= Math.max(1,9/this.scale)) {
            this.selected = best;
            if (this.scale < 8) {
                this.scale = 12;
                this.center = [x,y];
            }
        }
    }
    draw(replay, f, showWorld) {
        const ctx = this.ctx, s=this.scale, names=new Set(["Obstacles"]);
        const layer = showWorld ? replay.actual : replay.known;
        const pose = replay.pose(f.moved);
        const ratio = window.devicePixelRatio || 1;
        ctx.setTransform(ratio,0,0,ratio,0,0);
        ctx.fillStyle="#f6f6f4"; ctx.fillRect(0,0,this.w,this.h);
        ctx.translate(this.w/2,this.h/2); ctx.scale(s,s); ctx.translate(-this.center[0],-this.center[1]);
        ctx.imageSmoothingEnabled=false;
        ctx.drawImage(layer.canvas,0,0);
        this.searchLayer(f,layer,names);
        if (f.edit && (f.edit.kind !== "world" || showWorld)) {
            for (const [field,name] of [["block","Newly blocked"],["clear","Newly cleared"]]) {
                if (f.edit[field].length) names.add(name);
                ctx.fillStyle=ink[name];
                for (const i of f.edit[field]) ctx.fillRect(i%layer.width, Math.floor(i/layer.width),1,1);
            }
        }
        if (s >= 7) {
            const [left,top]=this.point(0,0), [right,bottom]=this.point(this.w,this.h);
            const x0=Math.max(0,Math.floor(left)), x1=Math.min(layer.width,Math.ceil(right));
            const y0=Math.max(0,Math.floor(top)), y1=Math.min(layer.canvas.height,Math.ceil(bottom));
            ctx.beginPath();
            for(let x=x0;x<=x1;x++){ctx.moveTo(x,y0);ctx.lineTo(x,y1);}
            for(let y=y0;y<=y1;y++){ctx.moveTo(x0,y);ctx.lineTo(x1,y);}
            ctx.strokeStyle="#55555548";ctx.lineWidth=.65/s;ctx.stroke();
        }
        if (replay.spec.scenario === "limited_visibility") {
            ctx.beginPath();ctx.arc(pose[0]+.5,pose[1]+.5,replay.spec.radius,0,2*Math.PI);
            ctx.strokeStyle=ink["Sensor radius"];ctx.lineWidth=1.4/s;ctx.setLineDash([5/s,4/s]);
            ctx.stroke();ctx.setLineDash([]);names.add("Sensor radius");
        }
        for (let i=0;i<f.moved;i++) {
            const move=replay.moves[i];
            this.curve(move.origin,this.controls[move.id],"Path traveled",1.8);
            names.add("Path traveled");
        }
        for (const step of (f.plan || []).slice(f.offset)) {
            const p=this.controls[step.id];
            if (step.cut !== 0) {this.curve(step.pose,p,"Planned path",1.7);names.add("Planned path");}
            if (step.cut !== null) {
                let from=0;
                if (step.cut) {
                    const [x,y]=p.trace[step.cut];
                    let min=Infinity;
                    p.curve.forEach(([a,b],i)=>{const d=(a-x)**2+(b-y)**2;if(d<min){min=d;from=i;}});
                }
                this.curve(step.pose,p,"Reused path",3,from);names.add("Reused path");
            }
        }
        if (f.candidate) {
            const [x,y,tx,ty,id,clear, , ,origin,motions] = f.candidate;
            const name = clear ? "Edge check" : "Blocked edge";
            ctx.save();
            if (!clear) ctx.setLineDash([4/s,2/s]);
            if (replay.spec.graph === "lattice") {
                this.curve([x,y],this.controls[id],name,2.5);
            } else {
                if (motions && motions.length) {
                    const layer = motions.length === 1 ? "Primitive trajectory" : "Possible primitives";
                    ctx.setLineDash([]);
                    for (const motion of motions)
                        this.curve(origin,this.controls[motion],layer,motions.length === 1 ? 2.2 : 1.2);
                    names.add(layer);
                    if (!clear) ctx.setLineDash([4/s,2/s]);
                }
                ctx.beginPath();
                ctx.moveTo(x+.5,y+.5);ctx.lineTo(tx+.5,ty+.5);
                ctx.lineWidth=2.5/s;ctx.strokeStyle=ink[name];ctx.stroke();
            }
            ctx.fillStyle=ink[name];
            ctx.beginPath();ctx.arc(tx+.5,ty+.5,2.5/s,0,2*Math.PI);ctx.fill();
            ctx.restore();
            names.add(name);
        } else if (s >= 7) this.inspect(replay,f,names);
        this.marker(replay.spec.start,"Start");names.add("Start");
        this.marker(replay.spec.goal,"Goal",true);names.add("Goal");
        if(f.event >= 0) {this.marker(pose,"Robot");names.add("Robot");}
        const signature = [...names].join();
        if (signature !== this.signature) {
            this.signature=signature;
            this.legend.replaceChildren(...Array.from(names,name=>{
                const span=document.createElement("span"), swatch=document.createElement("i");
                swatch.style.background=ink[name];span.append(swatch,name);return span;
            }));
        }
    }
}
