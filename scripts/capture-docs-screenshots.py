"""Capture docs/figures screenshots from the running Vite app."""

from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "figures"
URL = "http://localhost:5173/"


def shot(page, name):
    page.wait_for_timeout(450)
    path = OUT / name
    page.screenshot(path=str(path), type="png")
    print(f"saved {path.name}", flush=True)


def wait_ready(page):
    page.wait_for_function(
        """() => {
            const s = document.getElementById('status');
            return s && s.textContent.includes('就绪');
        }""",
        timeout=120_000,
    )


def wait_plan_done(page):
    page.wait_for_function(
        """() => {
            const busy = document.getElementById('busy');
            const fly = document.getElementById('c-fly');
            const status = document.getElementById('status');
            const overlayHidden = busy && busy.hidden;
            const flyOn = fly && !fly.disabled;
            const text = status ? status.textContent : '';
            const failed = text.includes('失败') || text.includes('未找到') || text.includes('未连到');
            return overlayHidden && (flyOn || failed);
        }""",
        timeout=240_000,
    )
    return page.evaluate(
        """() => ({
            status: document.getElementById('status')?.textContent || '',
            length: document.getElementById('m-length')?.textContent || '',
            clearance: document.getElementById('m-clearance')?.textContent || '',
            inside: document.getElementById('m-inside')?.textContent || '',
            flyEnabled: !document.getElementById('c-fly')?.disabled,
        })"""
    )


def wait_sim_time(page, t, timeout=180_000):
    page.wait_for_function(
        f"() => window.__app?.sim?.data?.time >= {t}",
        timeout=timeout,
    )


def wait_holding(page, timeout=240_000):
    page.wait_for_function(
        """() => {
            const status = document.getElementById('status')?.textContent || '';
            return window.__app?.holding === true || status.includes('定点悬停');
        }""",
        timeout=timeout,
    )


def metrics(page):
    return page.evaluate(
        """() => ({
            status: document.getElementById('status')?.textContent || '',
            length: document.getElementById('m-length')?.textContent || '',
            clearance: document.getElementById('m-clearance')?.textContent || '',
            time: document.getElementById('m-time')?.textContent || '',
            inside: document.getElementById('m-inside')?.textContent || '',
        })"""
    )


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(
            channel="msedge",
            headless=True,
            args=[
                "--use-angle=gl",
                "--enable-webgl",
                "--ignore-gpu-blocklist",
            ],
        )
        page = browser.new_page(viewport={"width": 1600, "height": 900}, device_scale_factor=1)
        page.goto(URL, wait_until="domcontentloaded", timeout=60_000)
        wait_ready(page)
        page.wait_for_timeout(800)
        shot(page, "01-idle-overview.png")

        page.click("#c-plan-asymmetric")
        info = wait_plan_done(page)
        print("asymmetric plan:", info, flush=True)
        if not info["flyEnabled"]:
            raise SystemExit(f"asymmetric planning failed: {info}")
        page.wait_for_timeout(600)
        shot(page, "02-planned-corridor.png")

        page.check("#c-tree")
        page.wait_for_timeout(400)
        shot(page, "03-corridor-rrt-tree.png")
        page.uncheck("#c-tree")

        page.click("#c-fly")
        wait_sim_time(page, 16.5)
        page.click("#c-pause")
        page.wait_for_timeout(350)
        print("mid-flight:", metrics(page), flush=True)
        shot(page, "04-flight-tracking.png")
        page.click("#c-pause")

        page.check("#c-follow")
        wait_holding(page)
        page.wait_for_timeout(1200)
        print("hover:", metrics(page), flush=True)
        shot(page, "05-follow-hover.png")

        page.uncheck("#c-follow")
        page.click("#c-reset")
        page.wait_for_timeout(400)
        page.click("#c-plan-symmetric")
        info = wait_plan_done(page)
        print("symmetric plan:", info, flush=True)
        if not info["flyEnabled"]:
            raise SystemExit(f"symmetric planning failed: {info}")
        page.wait_for_timeout(600)
        shot(page, "06-planned-symmetric.png")

        browser.close()


if __name__ == "__main__":
    main()
