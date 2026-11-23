"""Render the retained benchmark results with the full-pinned report template."""
import csv
import io
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean

from matplotlib import rc_context
from matplotlib.figure import Figure
from matplotlib.ticker import NullLocator

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1]))
from bench.visualization import BASES, METRICS, data_uri, export_link, figure_exports, measurements

TEMPLATE = ROOT.parents[1] / "bench/timing_template.html"
NAMES = {
    "astar": ("A*", "#3a5268"), "dstar_lite": ("D* Lite", "#046e72"),
    "mpgaa": ("MPGAA*", "#b34b23"), "dstar_lite_chain": ("D* Lite chain", "#76518a"),
    "mpgaa_chain": ("MPGAA* chain", "#607b2e"),
}
BASIC_VARIANTS = [(a, g) for a in ("astar", "dstar_lite", "mpgaa") for g in ("lattice", "mesh")]
VARIANTS = BASIC_VARIANTS + [(a, "mesh") for a in ("dstar_lite_chain", "mpgaa_chain")]
SCENARIOS = {"navigation": ("Navigation replanning", "Changed cells (%)"),
             "visibility": ("Limited visibility", "Visibility radius (cells)")}
PARAMETERS = {"navigation": "percentage of changed cells", "visibility": "visibility radius"}


def matched_series(rows):
    groups = defaultdict(dict)
    for row in rows:
        key = row["series"], row["map"], row["task"], row["repeat"], row["seed"]
        variant = row["algorithm"], row["graph"]
        if variant in groups[key]:
            raise ValueError(f"Duplicate result for {key}, {variant}")
        groups[key][variant] = row
    data = {scenario: defaultdict(lambda: defaultdict(list)) for scenario in SCENARIOS}
    for (series, *_), group in sorted(groups.items()):
        if set(group) != set(VARIANTS):
            raise ValueError(f"Missing variant in {series}")
        if any(row["status"] not in ("reached", "no_path") for row in group.values()):
            continue
        scenario, value = series.split("-")
        for variant, row in group.items():
            data[scenario][float(value)][variant].append({
                **{k: float(row[k]) for k in METRICS}, "expanded": int(row["expanded"]),
                "expanded_cells_sum": int(row["expanded_cells_sum"]), "journal": ROOT / row["journal"],
            })
    return data


def table(metric, base, columns, caption, filename, variants):
    buf = io.StringIO(newline="")
    writer = csv.writer(buf, delimiter=";")
    writer.writerow(["Algorithm", "Graph", "Condition", "Episodes", "Mean time (ms)", "Speedup", "Baseline"])
    html = ['<div class="scroll"><table><thead><tr><th rowspan="2">Algorithm</th><th rowspan="2">Graph</th>']
    html.extend(f'<th colspan="2">{heading}</th>' for heading, _, _ in columns)
    html.append('</tr><tr>' + '<th>ms</th><th>Speedup</th>' * len(columns) + '</tr></thead><tbody>')
    for a, g in variants:
        name = NAMES[a][0]
        html.append(f'<tr><th scope="row">{name}</th><td class="graph">{g}</td>')
        for heading, series, x in columns:
            n, ms, speedup = measurements(series, (a, g), metric, base, x)
            writer.writerow([name, g, heading, n, f"{ms:.8f}", f"{speedup:.8f}", BASES[base]])
            html.append(f'<td>{ms:.1f}</td><td>{speedup:.2f}×</td>')
        html.append('</tr>')
    html.append('</tbody></table></div>')
    link = export_link(data_uri("text/csv;charset=utf-8", buf.getvalue().encode("utf-8-sig")), filename + ".csv", "CSV ↓")
    return '<figure class="table-block">' + ''.join(html) + caption_html(caption, link) + '</figure>'


def caption_html(caption, links):
    return f'<figcaption><span>{caption}</span><span class="downloads">{links}</span></figcaption>'


def figure_html(fig, filename, title, caption):
    images, links = figure_exports(fig, filename)
    return f'<figure><img alt="{title}" src="{images["svg"]}">' + caption_html(caption, links) + '</figure>'


def lines(ax, xs, ys, variant):
    a, g = variant
    name, color = NAMES[a]
    mesh = g == "mesh"
    return ax.plot(xs, ys, color=color, lw=1.25, ls="--" if mesh else "-", marker="x" if mesh else "o",
                   ms=4.4 if mesh else 5.5, markerfacecolor="white", label=f"{name} · {g}")


def style_axis(ax):
    ax.spines[["right", "top"]].set_visible(False)
    ax.grid(axis="y", color="#c7c7c7", lw=.55, ls=":")
    ax.set_axisbelow(True)


def param_axis(ax, scenario, xs):
    if scenario == "navigation":
        ax.set_xscale("log")
    ax.set_xticks(xs, labels=[f"{x:g}" for x in xs])
    ax.xaxis.set_minor_locator(NullLocator())
    ax.set_xlabel(SCENARIOS[scenario][1])
    style_axis(ax)


def timing_chart(data, scenario, metric, base, number, variants, suffix):
    title, _ = SCENARIOS[scenario]
    series = data[scenario]
    xs = sorted(series)
    fig = Figure(figsize=(6.1, 4.0))
    ax = fig.subplots()
    fig.subplots_adjust(left=.115, right=.97, top=.705, bottom=.15)
    fig.suptitle(f"{METRICS[metric]} · {title}", y=.98, fontsize=13)
    ax.axhline(1, color="#929fa4", lw=1)
    bounds = []
    for variant in variants:
        ys = [measurements(series, variant, metric, base, x)[2] for x in xs]
        lines(ax, xs, ys, variant)
        bounds.extend(ys)
    lo, hi = min(bounds)*.83, max(bounds)*1.23
    ticks = [2.**i for i in range(math.ceil(math.log2(lo)), math.floor(math.log2(hi))+1)]
    if min(ticks) < .125:
        fig.subplots_adjust(left=.17)
    ax.set_yscale("log", base=2)
    ax.set_ylim(lo, hi)
    ax.set_yticks(ticks, labels=[f"{v:g}×" for v in ticks])
    ax.set_ylabel("Speedup relative to A*")
    param_axis(ax, scenario, xs)
    fig.legend(loc="upper center", bbox_to_anchor=(.5,.91), ncol=3, fontsize=8.5,
               handlelength=1.6, columnspacing=1.1, frameon=False)
    caption = f"<strong>Figure {number}.</strong> Speedup in {METRICS[metric].lower()} by {PARAMETERS[scenario]}."
    return figure_html(fig, f"{scenario}-{metric}-{base}{suffix}", f"{METRICS[metric]} · {title}", caption)


def search_diagnostics(data):
    samples = {}
    for scenario, series in data.items():
        samples[scenario] = {}
        for variant in VARIANTS:
            cells, latencies, ops = [], [], Counter()
            for point in series.values():
                for row in point[variant]:
                    with row["journal"].open(encoding="utf-8") as stream:
                        for line in stream:
                            result = json.loads(line)
                            if result["kind"] != "result":
                                continue
                            if result["event"] == 0:
                                cells.append(result["expanded_cells"])
                            else:
                                latencies.append(result["plan_ms"])
                                ops.update(result["open"])
                                ops["repair"] += sum(result["repair_queue"].values())
            samples[scenario][variant] = cells, latencies, ops
    return samples


def diagnostic_figures(data, samples, variants, suffix):
    sections = ['<section class="scenario diagnostics"><h2>Search diagnostics</h2>']
    captions = {
        "cells": "Distinct cells expanded during the initial search. Each point compares the same algorithm on both graphs. A point below the diagonal indicates fewer cells expanded on the mesh.",
        "queues": "Mean number of queue operations per episode during replanning. Repair counts all operations on the MPGAA* heuristic repair queue.",
        "latency": "Cumulative distribution of replanning time. At each time on the horizontal axis, the curve shows the fraction of replanning requests that finished within that time.",
        "expanded_cells_sum": "Mean number of cells expanded per episode. Each request counts distinct cell coordinates, and the episode total sums those counts.",
        "expanded": "Mean number of state expansions per episode, including repeated expansions of the same state.",
    }
    for number, (kind, caption) in enumerate(captions.items(), 5):
        fig = Figure(figsize=(10, 4.7 if kind == "queues" else 4.25), layout="constrained")
        axes = fig.subplots(1, 2, sharey=kind in ("cells", "queues", "latency"))
        for ax, (scenario, series) in zip(axes, data.items()):
            sample = samples[scenario]
            ax.set_title(SCENARIOS[scenario][0], fontsize=12)
            style_axis(ax)
            if kind == "cells":
                maximum = max(max(s[v][0]) for s in samples.values() for v in BASIC_VARIANTS)
                for a, marker in zip(("astar", "dstar_lite", "mpgaa"), ("o", "s", "^")):
                    name, color = NAMES[a]
                    ax.scatter(sample[a, "lattice"][0], sample[a, "mesh"][0], marker=marker,
                               s=20, facecolors="none", edgecolors=color, lw=.8, label=name)
                ax.plot([0, maximum], [0, maximum], ":", color=".45", zorder=0)
                ax.set_xscale("symlog", linthresh=10)
                ax.set_yscale("symlog", linthresh=10)
                ax.set(xlim=(-.3, maximum*1.3), ylim=(-.3, maximum*1.3), aspect="equal", xlabel="Cells expanded on the lattice")
                axes[0].set_ylabel("Cells expanded on the mesh")
                ax.legend(frameon=False, fontsize=8)
            elif kind == "queues":
                left = [0]*len(variants)
                keys = ("insert", "pop", "change", "erase", "peek", "repair")
                for key, color in zip(keys, ("#3a5268","#7794aa","#a2c1c4","#aaa39c","#ddc2aa","#b34b23")):
                    values = [sample[v][2][key]/len(sample[v][0])/1000 for v in variants]
                    ax.barh(range(len(values)), values, left=left, color=color, height=.64, label=key.capitalize())
                    left = [x+y for x,y in zip(left, values)]
                ax.set_yticks(range(len(variants)), labels=[f"{NAMES[a][0]} · {g}" for a,g in variants], fontsize=9)
                ax.set_xlabel("Queue operations per episode (thousands)")
            elif kind == "latency":
                for a, g in variants:
                    _, latency, _ = sample[a, g]
                    name, color = NAMES[a]
                    ax.ecdf(latency, color=color, lw=1.15, ls="--" if g=="mesh" else "-", label=f"{name} · {g}")
                ax.set_xscale("symlog", linthresh=.01)
                ax.set(xlabel="Replanning time (ms)", ylim=(0,1.02))
                axes[0].set_ylabel("Fraction of requests completed")
            else:
                xs = sorted(series)
                for v in variants:
                    lines(ax, xs, [mean(r[kind] for r in series[x][v]) for x in xs], v)
                param_axis(ax, scenario, xs)
                ax.set_ylim(bottom=0)
                axes[0].set_ylabel("Cells expanded per episode" if kind=="expanded_cells_sum" else "State expansions per episode")
        if kind != "cells":
            handles, labels = axes[0].get_legend_handles_labels()
            fig.legend(handles, labels, loc="outside upper center", ncol=3, fontsize=9, frameon=False)
        sections.append(figure_html(fig, f"diagnostic-{kind}{suffix}", caption, f"<strong>Figure {number}.</strong> {caption}"))
    return ''.join(sections)+'</section>'


def timing_section(data, base, variants, suffix):
    hidden = ' hidden' if base != "same" else ''
    parts = [f'<div data-base="{base}"{hidden}><div class="tables">']
    columns = [(SCENARIOS[s][0], series, None) for s, series in data.items()]
    for i, (metric, label) in enumerate(METRICS.items(), 1):
        parts.append(table(metric, base, columns, f"<strong>Table {i}.</strong> {label} per episode.",
                           f"{metric}-{base}{suffix}", variants))
    parts.append('</div>')
    for idx, (s, series) in enumerate(data.items()):
        title, xlabel = SCENARIOS[s]
        parts.append(f'<section class="scenario"><h2>{title}</h2><div class="charts">')
        for i, metric in enumerate(METRICS, 1):
            parts.append(timing_chart(data, s, metric, base, idx*2+i, variants, suffix))
        parts.append(f'</div><details><summary>Timing tables by {PARAMETERS[s]}</summary>')
        columns = [(f"{x:g}", series, x) for x in sorted(series)]
        for metric, label in METRICS.items():
            parts.append(table(metric, base, columns, f"{label} per episode. Columns show {xlabel.lower()}.",
                               f"{s}-{metric}-{base}{suffix}", variants))
        parts.append('</details></section>')
    return ''.join(parts)+'</div>'


def render(rows, data):
    samples = search_diagnostics(data)
    matched = sum(len(point["astar", "lattice"]) for series in data.values() for point in series.values())
    counts = Counter(r["status"] for r in rows)
    parts = [f'<div class="intro"><h1>Planning Time Comparison</h1><p class="meta">car16 · lattice and mesh · '
             f'{matched:,} matched task, repeat and scenario combinations</p></div>',
             '<div class="tools"><label for="planners">Planners</label><select id="planners">'
             f'<option value="all">All variants ({len(VARIANTS)})</option>'
             '<option value="basic">Basic algorithms only (6)</option></select>'
             '<label for="base">Baseline</label><select id="base">'
             + ''.join(f'<option value="{key}">{label}</option>' for key,label in BASES.items()) + '</select>'
             '<span>Speedup = mean A* time / mean planner time. Values above 1× indicate faster planning than A*.</span></div>',
             '<h2>Mean time per episode</h2>']
    with rc_context({"font.family":"serif", "font.serif":["Times New Roman","DejaVu Serif"],
                     "font.size":10, "svg.fonttype":"none", "axes.linewidth":.7}):
        for mode, variants in (("all", VARIANTS), ("basic", BASIC_VARIANTS)):
            suffix = "" if mode == "all" else "-basic"
            hidden = "" if mode == "all" else " hidden"
            parts.append(f'<div data-planners="{mode}"{hidden}>')
            for base in BASES:
                parts.append(timing_section(data, base, variants, suffix))
            parts.append(diagnostic_figures(data, samples, variants, suffix))
            parts.append('</div>')
    parts.append(f'<section class="scenario"><h2>Full experiment data</h2><p>Long profile · Release build · Labyrinth, Moscow_0_512 and 32room_000 · 18 tasks · 3 repeats · '
                 f'60 seconds per episode. Of {len(rows):,} episodes, {counts["reached"]:,} reached the goal, '
                 f'{counts["no_path"]:,} returned no path, {counts["timeout"]:,} timed out, {counts["error"]:,} ended in an error, '
                 f'and {counts["not_run"]:,} were skipped.</p>'
                 '<p>Both planner selections use the same tasks and repeats, for which all listed variants returned a result. '
                 'Each planner drives its own robot. Replanning time sums all requests after the first. '
                 'Total time includes graph initialization, planning and simulation. Up to four series ran concurrently '
                 'on separate CPU cores. Each series ran its variants sequentially.</p>')
    parts.append('<details><summary>Matched tasks and outcomes by condition</summary><div class="scroll">'
                 '<table><thead><tr><th>Scenario</th><th>Condition</th><th>Matched / 54</th>'
                 '<th>Reached</th><th>No path</th><th>Timeout</th><th>Error</th><th>Skipped</th></tr></thead><tbody>')
    for scenario, series in data.items():
        for value in sorted(series):
            tag = f"{scenario}-{value:g}"
            states = Counter(r["status"] for r in rows if r["series"] == tag)
            n = len(series[value]["astar", "lattice"])
            parts.append(f'<tr><th scope="row">{SCENARIOS[scenario][0]}</th><td>{value:g}</td><td>{n}</td>'
                         + ''.join(f'<td>{states[s]}</td>' for s in ("reached","no_path","timeout","error","not_run"))
                         + '</tr>')
    parts.append('</tbody></table></div></details><p>')
    for name, label in [("summary","All episodes"), ("means","Mean times and same-graph speedups"), ("outcomes","Outcomes")]:
        parts.append(export_link(data_uri("text/csv;charset=utf-8", (ROOT/(name+".csv")).read_bytes()), name+".csv", label+" ↓"))
        parts.append(' ')
    parts.append('</p></section>')
    template = TEMPLATE.read_text(encoding="utf-8")
    head, rest = template.split('<main>', 1)
    _, end = rest.split('</main>', 1)
    head = head.replace("</style>", "details{margin-top:12px}summary{cursor:pointer}details figure{margin:12px 0}"
                        ".table-block tbody tr:nth-child(n+7)>*{border-top:1px solid #ccc}</style>")
    planner_script = """<script>
const planners = document.getElementById("planners");
function showPlanners() {
    for (const panel of document.querySelectorAll("[data-planners]")) {
        panel.hidden = panel.dataset.planners !== planners.value;
    }
}
planners.addEventListener("change", showPlanners);
showPlanners();
</script>"""
    (ROOT/"timing.html").write_text(head+'<main>'+''.join(parts)+'</main>'+planner_script+end, encoding="utf-8")


def save_csv(name, rows):
    with (ROOT/name).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    rows, outcomes = [], []
    manifests, builds, conditions = set(), set(), set()
    for path in sorted(ROOT.glob("*/run.json")):
        meta = json.loads(path.read_text(encoding="utf-8"))
        series = path.parent.name
        manifests.add(meta["manifest_sha256"])
        builds.add(meta["binary_sha256"])
        conditions.add(series)
        expected = {(t["id"], str(i), a, g) for t in meta["tasks"]
                    for i in range(meta["repeats"]) for a, g in VARIANTS}
        with (path.parent/"summary.csv").open(newline="", encoding="utf-8") as stream:
            group = list(csv.DictReader(stream))
        keys = {(r["task"], r["repeat"], r["algorithm"], r["graph"]) for r in group}
        if (meta["profile"] != "long" or meta["repeats"] != 3 or len(meta["tasks"]) != 18 or
                set(map(tuple, meta["variants"])) != set(VARIANTS) or
                keys != expected or len(group) != len(expected) or
                dict(Counter(r["status"] for r in group)) != meta.get("outcomes")):
            raise ValueError(f"Incomplete campaign: {series}")
        for r in group:
            if r["journal"]:
                r["journal"] = series + "/" + r["journal"]
            rows.append(dict(series=series, **r))
        for a, g in VARIANTS:
            counts = Counter(r["status"] for r in group if (r["algorithm"], r["graph"]) == (a,g))
            outcomes.append(dict(series=series, algorithm=a, graph=g,
                                 **{s: counts[s] for s in ("reached","no_path","timeout","error","not_run")}))
    required = {"navigation-"+p for p in ("0.1","0.5","1","5")} | {"visibility-"+r for r in ("25","50","100","200")}
    if conditions != required or len(manifests) != 1 or len(builds) != 1:
        raise ValueError("Incompatible scenarios, task manifests or binaries")
    data = matched_series(rows)
    means = []
    for scenario, series in data.items():
        for point in sorted(series):
            for a,g in VARIANTS:
                for metric in METRICS:
                    n, ms, ratio = measurements(series, (a,g), metric, "same", point)
                    means.append(dict(scenario=scenario, parameter=point, algorithm=a, graph=g,
                                      metric=metric, matched=n, mean_ms=ms, same_graph_speedup=ratio))
    save_csv("summary.csv", rows)
    save_csv("outcomes.csv", outcomes)
    save_csv("means.csv", means)
    render(rows, data)


if __name__ == "__main__":
    main()
