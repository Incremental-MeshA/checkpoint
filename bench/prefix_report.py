"""Summarize matched MPGAA* episodes and compare answers on identical planning requests."""
import argparse
import csv
import hashlib
import itertools
import json
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean
import os

VARIANTS = [("mpgaa", "lattice"), ("mpgaa", "mesh"), ("mpgaa_local", "mesh"),
            ("mpgaa_chain", "mesh"), ("mpgaa_local_chain", "mesh")]
COMPLETE = {"reached", "no_path"}


def write_csv(path, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def timing_page(dest, means, outcomes, binary):
    from matplotlib.figure import Figure
    from .visualization import figure_exports

    labels = ["Lattice MPGAA*", "Mesh MPGAA*", "Локальный ремонт", "Сжатие цепочек", "Оба изменения"]
    colors = ["#68459b", "#585858", "#9e4b09", "#136bad", "#008058"]
    body = ["<!doctype html><html lang='ru'><meta charset='utf-8'><title>Эксперимент MPGAA*</title>",
            "<style>body{font:16px/1.5 system-ui;color:#192228;max-width:1100px;margin:2em auto;padding:0 1em}"
            "img{max-width:100%}table{border-collapse:collapse}td,th{padding:.4em .8em;text-align:right;"
            "border-bottom:1px solid #ccd2d6}th:first-child{text-align:left}a{margin-right:.9em}</style>",
            "<h1>Локальный ремонт и сжатие цепочек MPGAA*</h1>",
            f"<p>Release {binary[:12]} · {sum(r['attempted'] for r in outcomes)} эпизодов · лимит 60 с.</p>",
            "<p>Ускорение равно отношению средних времён исходного mesh-MPGAA* и выбранного варианта. "
            "В каждой точке учтены одинаковые задачи и повторы с исходом reached или no_path у всех пяти вариантов. "
            "Перепланирование суммируется за эпизод. Значения выше 1 означают выигрыш.</p>",
            '<p><a href="means.csv">Средние, мс и счётчики</a><a href="outcomes.csv">Исходы</a>'
            '<a href="episodes.csv">Все эпизоды</a><a href="checks.csv">Сверка запросов</a></p>']
    for prefix, title, xlabel in [("navigation", "Navigation replanning", "Изменено клеток, %"),
                                  ("visibility", "Limited visibility", "Радиус видимости, клеток")]:
        data = {(float(r["series"].split("-")[1]), r["algorithm"], r["graph"]): r
                for r in means if r["series"].startswith(prefix)}
        xs = sorted({key[0] for key in data})
        body.append(f"<h2>{title}</h2>")
        fig = Figure(figsize=(10,4.4), layout="constrained")
        axes = fig.subplots(1,2)
        for ax, metric, heading in zip(axes, ("total_ms","replan_ms"), ("Весь эпизод","Перепланирование")):
            for (a,g), label, color in zip(VARIANTS, labels, colors):
                ax.plot(xs, [data[x,a,g]["mesh_speedup_"+metric] for x in xs],
                        marker="o", label=label, color=color)
            if prefix == "navigation":
                ax.set_xscale("log")
            ax.set_yscale("log", base=2)
            ax.set_xticks(xs, labels=[f"{x:g}" for x in xs])
            ax.set(title=heading, xlabel=xlabel, ylabel="Ускорение, ×")
            ax.grid(alpha=.25)
        axes[0].legend(fontsize=8)
        images, links = figure_exports(fig, prefix+"-mpgaa")
        body.append(f'<figure><img src="{images["svg"]}" alt="Ускорение относительно mesh-MPGAA*"><figcaption>{links}</figcaption></figure>')
        body.append("<p>Среднее суммарное время перепланирования за эпизод, мс.</p><table><thead><tr><th>"
                    +xlabel+"</th><th>Пар</th>"+"".join(f"<th>{s}</th>" for s in labels)+"</tr></thead><tbody>")
        for x in xs:
            n = data[x,"mpgaa","mesh"]["matched"]
            values = [data[x,a,g]["replan_ms"] for a,g in VARIANTS]
            body.append(f"<tr><th>{x:g}</th><td>{n}</td>"+"".join(f"<td>{v:.2f}</td>" for v in values)+"</tr>")
        body.append("</tbody></table>")
    (dest/"timing.html").write_text("\n".join(body)+"</html>", encoding="utf-8")


def read_journal(path, row, answers):
    # All variants in this task/repeat start with the same original map and goal.
    # Equal edit histories and poses identify equivalent graph queries even if
    # earlier returned routes differ. Different histories are kept separate.
    history = hashlib.sha256()
    work = Counter()
    variant = (row["algorithm"], row["graph"])
    with path.open(encoding="utf-8") as stream:
        first = json.loads(next(stream))
        assert (first["task"]["id"], first["seed"]) == (row["task"], int(row["seed"]))
        for line in stream:
            record = json.loads(line)
            if record["kind"] == "request":
                if record["block"] or record["clear"]:
                    edits = [sorted(record["block"]), sorted(record["clear"])]
                    history.update(json.dumps(edits, separators=(",", ":")).encode() + b"\n")
                key = (history.digest(), tuple(record["pose"]))
            elif record["kind"] == "result":
                value = (record["outcome"], record["cost"])
                found = answers[key]
                if found and next(iter(found.values())) != value:
                    raise ValueError(f"Cost/reachability mismatch at {path}, event {record['event']}: {found} vs {value}")
                found[variant] = value
                if record["event"] > 0:
                    for field in ("search_expanded", "repair_expanded", "local_repair_steps"):
                        work[field] += record.get(field, 0)
                    work["repair_pops"] += record["repair_queue"]["pop"]
        assert record["status"] == row["status"]
    return work


def summarize(campaign, dest):
    dest.mkdir(parents=True, exist_ok=True)
    episodes, means, outcomes, comparisons = [], [], [], []
    binaries = set()
    manifests = {}
    for manifest in sorted(campaign.glob("*/run.json")):
        meta = json.loads(manifest.read_text())
        series = manifest.parent.name
        manifests[series] = {k: meta[k] for k in ("binary_sha256", "manifest_sha256", "build", "cpu", "os",
                            "profile", "repeats", "episode_budget_seconds", "wall_seconds", "outcomes")}
        binaries.add(meta["binary_sha256"])
        with (manifest.parent/"summary.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        groups = defaultdict(dict)
        expected = set(itertools.product([t["id"] for t in meta["tasks"]], range(meta["repeats"]), VARIANTS))
        for row in rows:
            variant = (row["algorithm"], row["graph"])
            expected.remove((row["task"], int(row["repeat"]), variant))
            groups[row["task"], row["repeat"], row["seed"]][variant] = row
        assert not expected and dict(Counter(r["status"] for r in rows)) == meta["outcomes"]
        shared = fully_shared = 0
        paired = []
        for group in groups.values():
            assert set(group) == set(VARIANTS)
            answers = defaultdict(dict)
            for variant, row in group.items():
                path = manifest.parent/row["journal"]
                work = read_journal(path, row, answers)
                for field in ("search_expanded", "repair_expanded", "local_repair_steps", "repair_pops"):
                    row["replan_" + field] = work[field]
                episodes.append(dict(series=series, **{**row, "journal": os.path.relpath(path, dest)}))
            shared += sum(len(v) >= 2 for v in answers.values())
            fully_shared += sum(len(v) == len(VARIANTS) for v in answers.values())
            if all(r["status"] in COMPLETE for r in group.values()):
                paired.append(group)
        comparisons.append(dict(series=series, shared_queries=shared, all_five_queries=fully_shared,
                                matched_episodes_per_variant=len(paired)))
        for a,g in VARIANTS:
            counts = Counter(r["status"] for r in rows if (r["algorithm"],r["graph"]) == (a,g))
            outcomes.append(dict(series=series,algorithm=a,graph=g,attempted=sum(counts.values()),
                                 reached=counts["reached"],no_path=counts["no_path"],timeout=counts["timeout"],
                                 error=counts["error"],not_run=counts["not_run"]))
            report = dict(series=series, algorithm=a, graph=g, matched=len(paired))
            for metric in ("first_ms","replan_ms","total_ms","replan_search_expanded","replan_repair_expanded",
                           "replan_repair_pops","replan_local_repair_steps"):
                report[metric] = mean(float(group[a,g][metric]) for group in paired)
            for metric in ("replan_ms","total_ms"):
                base = mean(float(group["mpgaa","mesh"][metric]) for group in paired)
                report["mesh_speedup_"+metric] = base/report[metric]
            means.append(report)
    assert len(binaries) == 1 and len(comparisons) == 9
    (dest/"series.json").write_text(json.dumps(manifests, indent=2)+"\n", encoding="utf-8")
    for name, items in [("episodes",episodes),("means",means),("outcomes",outcomes),("checks",comparisons)]:
        write_csv(dest/(name+".csv"),items)
    timing_page(dest, means, outcomes, next(iter(binaries)))


if __name__ == "__main__":
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("campaign",type=Path)
    cli.add_argument("output",type=Path)
    args=cli.parse_args()
    summarize(args.campaign, args.output)
