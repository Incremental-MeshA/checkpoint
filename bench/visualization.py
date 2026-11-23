"""Build the standalone timing visualization from matched benchmark series."""
import argparse
import base64
import csv
import io
import json
import math
from html import escape
from pathlib import Path
from statistics import mean

from matplotlib import rc_context
from matplotlib.figure import Figure
from matplotlib.ticker import FixedLocator, NullLocator

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = {
    "navigation_replanning": ("Navigation replanning", "Changed cells (%)", "percent",
                              ("quick-0.1", "quick-1", "quick-10")),
    "limited_visibility": ("Limited visibility", "Visibility radius (cells)", "radius",
                           ("visibility-25", "visibility-50", "visibility-100")),
}
METRICS = {"total_ms": "Total time", "replan_ms": "Replanning time"}
ALGORITHMS = {"astar": ("Repeated A*", "#3a5268"),
              "dstar_lite": ("D* Lite", "#046e72"), "mpgaa": ("MPGAA*", "#b34b23")}
GRAPHS = ("lattice", "mesh")
ORIGINAL_VARIANTS = [(a, g) for a in ALGORITHMS for g in GRAPHS]
ALGORITHMS.update({"dstar_lite_chain": ("D* Lite (chains)", "#6d4b93"),
                   "mpgaa_chain": ("MPGAA* (chains)", "#926a08")})
BASES = {"same": "A* on the same graph", "lattice": "Lattice A* for all algorithms"}


def number(value, digits):
    return f"{value:.{digits}f}"


def load_series(results):
    """Each point uses exactly the same completed task/repeat/seed keys across variants."""
    data = {}
    expected_variants = None
    metadata = [(path.parent, json.loads(path.read_text(encoding="utf-8")))
                for path in sorted(results.glob("*/run.json"))]
    for scenario, (_, _, param, _) in SCENARIOS.items():
        series = {}
        for folder, meta in metadata:
            if meta["config"].get("scenario", "navigation_replanning") != scenario:
                continue
            value = float(meta[param])
            variants = [tuple(v) for v in meta.get("variants", ORIGINAL_VARIANTS)]
            if expected_variants is not None and set(variants) != expected_variants:
                raise ValueError(f"Mixed planner cohorts in {results}")
            expected_variants = set(variants)
            groups = {variant: {} for variant in variants}
            with (folder / "summary.csv").open(newline="", encoding="utf-8-sig") as stream:
                for row in csv.DictReader(stream):
                    if row["status"] not in ("reached", "no_path"):
                        continue
                    group = groups[row["algorithm"], row["graph"]]
                    key = row["map"], row["task"], row["repeat"], row["seed"]
                    if key in group:
                        raise ValueError(f"Duplicate episode in {folder}: {key}")
                    times = {metric: float(row[metric]) for metric in METRICS}
                    if any(not math.isfinite(t) or t < 0 for t in times.values()):
                        raise ValueError(f"Invalid time in {folder}: {key}")
                    times['journal'] = folder / row['journal']
                    for field in ("expanded", "expanded_cells_sum"):
                        times[field] = int(row[field])
                    group[key] = times
            matched = sorted(set.intersection(*(set(group) for group in groups.values())))
            if not matched or value in series:
                raise ValueError(f"Missing matched episodes or duplicate parameter in {folder}")
            series[value] = {variant: [group[k] for k in matched] for variant, group in groups.items()}
        if not series:
            raise ValueError(f"No measured series for {scenario} in {results}")
        data[scenario] = dict(sorted(series.items()))
    return data


def measurements(scenario, variant, metric, base, value=None):
    points = scenario.values() if value is None else [scenario[value]]
    baseline = ("astar", variant[1] if base == "same" else "lattice")
    elapsed = [r[metric] for point in points for r in point[variant]]
    reference = mean(r[metric] for point in points for r in point[baseline])
    algorithm_mean = mean(elapsed)
    if reference == 0 or algorithm_mean == 0:
        raise ValueError("Speedup plots require positive mean times for the baseline and algorithm")
    return len(elapsed), algorithm_mean, reference / algorithm_mean


def data_uri(mime, content):
    return f"data:{mime};base64," + base64.b64encode(content).decode("ascii")


def export_link(uri, name, label):
    return f'<a class="export" href="{uri}" download="{name}" aria-label="Download {name}">{label}</a>'


def make_table(data, metric, base):
    output = io.StringIO(newline="")
    writer = csv.writer(output, delimiter=";")
    writer.writerow(["Algorithm", "Graph", "Scenario", "Episodes", METRICS[metric] + ", ms",
                     "Speedup (mean A* time / mean algorithm time)", "Baseline"])
    body = []
    first_series = next(iter(data.values()))
    for algo, graph in next(iter(first_series.values())):
        name, _ = ALGORITHMS[algo]
        cells = []
        for key, series in data.items():
            n, ms, ratio = measurements(series, (algo, graph), metric, base)
            cells.append(f'<td>{number(ms, 1)}</td><td class="ratio">{number(ratio, 2)}×</td>')
            writer.writerow([name, graph, SCENARIOS[key][0], n, number(ms, 8), number(ratio, 8), BASES[base]])
        body.append(f'<tr><th scope="row">{name}</th>'
                    f'<td class="graph">{graph}</td>{"".join(cells)}</tr>')
    link = export_link(data_uri("text/csv;charset=utf-8", output.getvalue().encode("utf-8-sig")),
                       f"{metric}-{base}.csv", "CSV ↓")
    headings = "".join(f'<th scope="colgroup" colspan="2">{item[0]}</th>' for item in SCENARIOS.values())
    units = '<th scope="col">ms</th><th scope="col">Speedup</th>' * len(SCENARIOS)
    table_number = list(METRICS).index(metric) + 1
    return f'''<figure class="table-block">
    <div class="scroll"><table aria-label="{METRICS[metric]} per episode"><thead>
    <tr><th rowspan="2" scope="col">Algorithm</th><th rowspan="2" scope="col">Graph</th>{headings}</tr>
    <tr>{units}</tr></thead><tbody>{"".join(body)}</tbody></table></div>
    <figcaption><span><strong>Table {table_number}.</strong> {METRICS[metric]} per episode.</span>
    <span class="downloads">{link}</span></figcaption></figure>'''


def figure_exports(fig, filename):
    images = {}
    for extension, mime in [("svg", "image/svg+xml"), ("png", "image/png")]:
        stream = io.BytesIO()
        fig.savefig(stream, format=extension, dpi=180, metadata={"Creator": "car16 bench"} if extension=="svg" else {})
        images[extension] = data_uri(mime, stream.getvalue())
    links = "".join(export_link(uri, f"{filename}.{ext}", ext.upper()+" ↓") for ext, uri in images.items())
    return images, links


def make_chart(data, scenario, metric, base):
    title, xlabel, _, _ = SCENARIOS[scenario]
    series = data[scenario]
    xs = list(series)
    fig = Figure(figsize=(6.4, 4.15), facecolor="white")
    ax = fig.subplots()
    fig.subplots_adjust(left=.145, right=.966, bottom=.18, top=.73)
    fig.text(.55, .962, f"{METRICS[metric]} · {title}", fontsize=12, ha="center")
    ys = []
    ax.axhline(1, color="#929fa4", linewidth=1)
    for algo, graph in next(iter(series.values())):
        name, color = ALGORITHMS[algo]
        values = [measurements(series, (algo, graph), metric, base, x)[2] for x in xs]
        ys.extend(values)
        mesh = graph == "mesh"
        ax.plot(xs, values, color=color, linewidth=1.25, linestyle="--" if mesh else "-",
                marker="x" if mesh else "o", markersize=4.5 if mesh else 6.5, markeredgewidth=1.15,
                markerfacecolor="white", label=f"{'A*' if algo == 'astar' else name} · {graph}",
                zorder=4 if mesh else 3)
    ax.set_yscale("log", base=2)
    ax.set_ylim(min(ys)*.83, max(ys)*1.23)
    ticks = [2.0**i for i in range(math.ceil(math.log2(min(ys)*.83)),
                                    math.floor(math.log2(max(ys)*1.23))+1)]
    ax.set_yticks(ticks, labels=[f"{v:g}×" for v in ticks])
    if scenario == "navigation_replanning":
        ax.set_xscale("log")
    ax.margins(x=.07)
    ax.xaxis.set_major_locator(FixedLocator(xs))
    ax.set_xticklabels([f"{x:g}" for x in xs])
    ax.xaxis.set_minor_locator(NullLocator())
    ax.yaxis.set_minor_locator(NullLocator())
    ax.set_xlabel(xlabel, labelpad=7)
    ax.set_ylabel("Speedup (A* / algorithm)")
    ax.grid(axis="y", color="#d2d2d2", linewidth=.55, linestyle=":")
    for edge in ("right", "top"):
        ax.spines[edge].set_visible(False)
    for edge in ("bottom", "left"):
        ax.spines[edge].set_color("#424242")
    ax.tick_params(length=3, width=.7, pad=4)
    ax.legend(loc="lower center", bbox_to_anchor=(.5, 1.025), ncol=3, fontsize=8.5,
              frameon=False, handlelength=2, columnspacing=1.25)
    images, links = figure_exports(fig, f"{scenario}-{metric}-{base}")
    # Reuse the displayed SVG for downloading; no browser-side renderer or network required.
    alt = escape(f"{METRICS[metric]} relative to A*. {title}. {xlabel}: " +
                 ", ".join(f"{x:g}" for x in xs))
    figure_number = list(SCENARIOS).index(scenario)*len(METRICS) + list(METRICS).index(metric) + 1
    parameter = "percentage of changed cells" if scenario == "navigation_replanning" else "visibility radius"
    caption = f"Speedup in {METRICS[metric].lower()} by {parameter}."
    return (f'<figure><img alt="{alt}" src="{images["svg"]}"><figcaption>'
            f'<span><strong>Figure {figure_number}.</strong> {caption}</span>'
            f'<span class="downloads">{links}</span></figcaption></figure>')


def diagnostic_data(series):
    samples = {v: {"cells": [], "latency": [], "ops": [0]*6} for v in next(iter(series.values()))}
    for point in series.values():
        for variant, episodes in point.items():
            sample = samples[variant]
            for episode in episodes:
                with episode["journal"].open(encoding="utf-8") as stream:
                    for line in stream:
                        record = json.loads(line)
                        if record["kind"] != "result":
                            continue
                        if record["event"] == 0:
                            sample["cells"].append(record["expanded_cells"])
                            continue
                        sample["latency"].append(record["plan_ms"])
                        for i, key in enumerate(("insert", "pop", "change", "erase", "peek")):
                            sample["ops"][i] += record["open"][key]
                        sample["ops"][5] += sum(record["repair_queue"].values())
    return samples


def diagnostics(data):
    datasets = {key: diagnostic_data(series) for key, series in data.items()}
    figures = []
    for number, (kind, caption) in enumerate((
        ("cells", "Distinct cells expanded on the initial plan. Each point pairs lattice and mesh for the same task, seed and condition; points below the diagonal favour mesh."),
        ("queues", "Mean queue operations per episode during replanning. The repair segment includes all operations on the MPGAA* repair queue."),
        ("latency", "Cumulative distribution of replanning latency. Each request after the initial plan contributes one observation."),
    ), 5):
        fig = Figure(figsize=(9.6, 3.6), layout="constrained")
        axes = fig.subplots(1, 2, sharey=True)
        for ax, (scenario, samples) in zip(axes, datasets.items()):
            ax.set_title(SCENARIOS[scenario][0], fontsize=12)
            ax.spines[["right", "top"]].set_visible(False)
            ax.grid(axis="y", linestyle=":", linewidth=.5, color="#cccccc")
            ax.set_axisbelow(True)
            if kind == "cells":
                for (algo, (name, color)), marker in zip(ALGORITHMS.items(), ("o", "s", "^")):
                    x = samples[algo, "lattice"]["cells"]
                    y = samples[algo, "mesh"]["cells"]
                    ax.scatter(x, y, s=20, facecolors="none", edgecolors=color,
                               marker=marker, linewidths=.8, label=name)
                maximum = max(max(sample["cells"]) for group in datasets.values() for sample in group.values())
                # The linear range around zero retains isolated endpoints.
                ax.set_xscale("symlog", linthresh=10)
                ax.set_yscale("symlog", linthresh=10)
                ax.plot([0, maximum], [0, maximum], ":", color=".45", zorder=0)
                ax.set(xlim=(-.3, maximum*1.3), ylim=(-.3, maximum*1.3), xlabel="Lattice cells")
                ax.set_aspect("equal", adjustable="box")
                axes[0].set_ylabel("Mesh cells")
                ax.legend(fontsize=8, frameon=False)
            elif kind == "queues":
                left = [0]*len(samples)
                for i, (name, color) in enumerate(zip(
                        ("Insert", "Pop", "Change key", "Erase", "Peek", "Repair"),
                        ("#3a5268", "#7794aa", "#a2c1c4", "#aaa39c", "#ddc2aa", "#b34b23"))):
                    values = [samples[v]["ops"][i]/len(samples[v]["cells"])/1000 for v in samples]
                    ax.barh(range(len(values)), values, left=left, height=.64, color=color, label=name)
                    left = [a+b for a, b in zip(left, values)]
                ax.set_yticks(range(len(samples)), labels=[f"{ALGORITHMS[a][0]} · {g}" for a,g in samples])
                ax.set_xlabel("Queue operations per episode (thousands)")
                ax.legend(loc="upper center", bbox_to_anchor=(.5,-.22), ncol=3, fontsize=8, frameon=False)
            else:
                for (algo, graph), sample in samples.items():
                    values = sorted(sample["latency"])
                    if not values:
                        continue
                    name, color = ALGORITHMS[algo]
                    ax.step([values[0], *values], [0, *(100*i/len(values) for i in range(1,len(values)+1))],
                            where="post", color=color, linewidth=1.2, ls="-" if graph=="lattice" else "--",
                            label=f"{'A*' if algo=='astar' else name} · {graph}")
                ax.set_xscale("symlog", linthresh=.01)
                ax.set(xlabel="Replanning latency (ms)", ylim=(0,102))
                axes[0].set_ylabel("Requests completed (%)")
                ax.legend(loc="lower right", fontsize=8, frameon=False)
        images, links = figure_exports(fig, f"diagnostic-{kind}")
        figures.append(f'<figure><img alt="{caption}" src="{images["svg"]}"><figcaption>'
                       f'<span><strong>Figure {number}.</strong> {caption}</span>'
                       f'<span class="downloads">{links}</span></figcaption></figure>')
    for number, field, label, detail in (
        (8, "expanded_cells_sum", "Expanded cells", "Each cell is counted once per planning request."),
        (9, "expanded", "State expansions", "Repeated expansions of a state count separately."),
    ):
        fig = Figure(figsize=(9.6, 3.8))
        axes = fig.subplots(1, 2)
        fig.subplots_adjust(left=.11, right=.98, top=.71, bottom=.16, wspace=.33)
        for ax, (scenario, series) in zip(axes, data.items()):
            title, xlabel, _, _ = SCENARIOS[scenario]
            xs = sorted(series)
            for algo, graph in next(iter(series.values())):
                name, color = ALGORITHMS[algo]
                values = [mean(row[field] for row in series[x][algo, graph]) for x in xs]
                mesh = graph == "mesh"
                ax.plot(xs, values, color=color, lw=1.25, linestyle="--" if mesh else "-",
                        marker="x" if mesh else "o", markerfacecolor="white",
                        label=f"{name} · {graph}")
            if scenario == "navigation_replanning":
                ax.set_xscale("log")
            ax.set_xticks(xs, labels=[f"{x:g}" for x in xs])
            ax.xaxis.set_minor_locator(NullLocator())
            ax.set(title=title, xlabel=xlabel, ylim=(0, None))
            ax.grid(axis="y", linewidth=.45, alpha=.35)
            ax.spines[["top", "right"]].set_visible(False)
        axes[0].set_ylabel(f"{label} per episode")
        fig.legend(*axes[0].get_legend_handles_labels(), loc="upper center", ncol=3, frameon=False)
        images, links = figure_exports(fig, field)
        caption = f"Mean {label.lower()} per episode, including the initial plan and all replans. {detail}"
        figures.append(f'<figure><img alt="{caption}" src="{images["svg"]}"><figcaption>'
                       f'<span><strong>Figure {number}.</strong> {caption}</span>'
                       f'<span class="downloads">{links}</span></figcaption></figure>')
    return '<section class="scenario diagnostics"><h2>Search diagnostics</h2>' + "".join(figures) + '</section>'


def render(results, destination):
    data = load_series(results)
    sections = []
    with rc_context({"font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
                     "font.size": 10, "svg.fonttype": "none", "axes.linewidth": .7}):
        for base in BASES:
            tables = "".join(make_table(data, metric, base) for metric in METRICS)
            scenarios = []
            for key, (title, _, _, _) in SCENARIOS.items():
                charts = "".join(make_chart(data,key,metric,base) for metric in METRICS)
                scenarios.append(f'<section class="scenario"><h2>{title}</h2><div class="charts">{charts}</div></section>')
            hidden = "" if base == "same" else " hidden"
            sections.append(f'<div data-base="{base}"{hidden}><div class="tables">{tables}</div>'
                            + "".join(scenarios) + "</div>")
        sections.append(diagnostics(data))
    total = sum(len(rows) for series in data.values() for point in series.values() for rows in point.values())
    template = Path(__file__).with_name("timing_template.html").read_text(encoding="utf-8")
    html = template.replace("{{COUNT}}", str(total))
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(html.replace("{{CONTENT}}", "".join(sections)), encoding="utf-8")


if __name__ == "__main__":
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--results", type=Path, default=ROOT/"results")
    cli.add_argument("--output", type=Path, default=ROOT/"results"/"timing.html")
    args = cli.parse_args()
    render(args.results, args.output)
    print(args.output.resolve())
