#!/usr/bin/env python3
"""Runs on the CI host: screenshots of the web UI while the simulation test runs."""
import asyncio
import os
import sys
import time

from playwright.async_api import async_playwright

OUT = sys.argv[1] if len(sys.argv) > 1 else 'ci_out'


async def main():
    deadline = time.time() + 1800
    while not os.path.exists(os.path.join(OUT, 'ready')) and time.time() < deadline:
        await asyncio.sleep(2)
    async with async_playwright() as p:
        exe = '/usr/bin/google-chrome' if os.path.exists('/usr/bin/google-chrome') else None
        browser = await p.chromium.launch(executable_path=exe, args=['--no-sandbox'])
        desk = await browser.new_page(viewport={'width': 1366, 'height': 900})
        mobile = await browser.new_page(viewport={'width': 390, 'height': 844}, device_scale_factor=2,
                                        is_mobile=True, has_touch=True)
        errors = []
        desk.on('pageerror', lambda e: errors.append(str(e)))
        for page in (desk, mobile):
            await page.goto('http://localhost:8000/')
        await asyncio.sleep(6)
        await desk.screenshot(path=os.path.join(OUT, 'ui_desktop_0_idle.png'))
        await mobile.screenshot(path=os.path.join(OUT, 'ui_mobile_0_idle.png'))
        i = 1
        while not os.path.exists(os.path.join(OUT, 'result.json')) and time.time() < deadline:
            await asyncio.sleep(20)
            await desk.screenshot(path=os.path.join(OUT, 'ui_desktop_%d.png' % i))
            i += 1
        await asyncio.sleep(2)
        await desk.screenshot(path=os.path.join(OUT, 'ui_desktop_final.png'))
        with open(os.path.join(OUT, 'ui_errors.txt'), 'w') as f:
            f.write('\n'.join(errors))
        await browser.close()


asyncio.run(main())
