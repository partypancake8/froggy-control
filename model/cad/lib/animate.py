"""Headless turntable animation: the spinning set (every component in module
"rotor" plus every component built with meta spins=True: ring and carrier
LEDs, Qi RX coil and board, Pi Zero, LED driver, index magnets, motor shaft and
jaw coupling) rotates about the Z axis through 360 deg while the frame,
plinth, drive base, electrical bay and cage stay still.

Studio look: low camera (elevation ELEV deg, iso azimuth), three-point lights
(key above-front, fill opposite and lower, rim behind), smooth shading split
at sharp edges, SSAA anti-aliasing, no caption on the frames. One PyVista
off-screen plotter, actors rotated per frame (no re-tessellation).

Outputs (stem drum_spin): .gif (primary, palette-optimised, one revolution
looping forever, GIF_W px wide), .mp4 (H.264 yuv420p at full px, `loops`
revolutions) and _sheet.png (0/90/180/270 deg, angle labelled)."""
import os
import shutil
import subprocess
import time

import numpy as np

from lib import mesh as MS
from lib.render import _pv, _pv_mesh

ELEV = 24.0          # camera elevation above horizontal, deg: lowest of 24/27/30 keeping the disc frog >= 400 px
                     # readable at 1000 px in every frame (18 hid it behind the clamp ring)
WATCH = ("v_frog_disc_body", "v_frog_disc_eyes")   # parts whose per-frame visibility is measured
AZIM = 45.0          # camera azimuth from +X toward +Y, deg (same direction as view_isometric)
GIF_W = 1200
BACKGROUNDS = {
    "warm_grey": dict(color=(0.86, 0.85, 0.83)),
    "gradient": dict(color=(0.17, 0.20, 0.25), top=(0.88, 0.89, 0.90)),   # dark slate bottom, light grey top
}


def spinning(A):
    return [n for n, p in A.parts.items() if p.module == "rotor" or p.meta.get("spins")]


def _encoder():
    exe = shutil.which("ffmpeg")
    if exe:
        return exe, "system"
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe(), "imageio-ffmpeg"
    except Exception:  # noqa: BLE001
        return None, None


def _unit(az, el):
    a, e = np.radians(az), np.radians(el)
    return np.array((np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)))


def _frame_camera(pts, focus, view_dir, view_angle, margin=1.10):
    """Perspective camera along view_dir, pulled back until every point fits
    in the view cone with a small margin, then re-aimed so the projected
    extent is centred (focus keeps its height as the pivot only)."""
    up = np.array((0.0, 0.0, 1.0))
    d = view_dir / np.linalg.norm(view_dir)
    right = np.cross(-d, up)
    right /= np.linalg.norm(right)
    cup = np.cross(right, -d)
    t = np.tan(np.radians(view_angle / 2)) / margin
    for _ in range(3):
        r = pts - focus
        along = r @ d
        dist = (np.maximum(np.abs(r @ right), np.abs(r @ cup)) / t + along).max()
        eye = focus + d * dist
        q = pts - eye
        z = q @ -d
        sx, sy = (q @ right) / z, (q @ cup) / z
        # shift the aim so the projected box is centred, at the focus depth
        focus = focus + right * 0.5 * (sx.max() + sx.min()) * dist + cup * 0.5 * (sy.max() + sy.min()) * dist
    return [tuple(focus + d * dist), tuple(focus), tuple(up)]


def _lights(pv, focus, R):
    f = np.asarray(focus)
    spec = (("key", AZIM + 35, 50, 0.85), ("fill", AZIM - 80, 15, 0.40), ("rim", AZIM + 180, 40, 0.55))
    out = []
    for name, az, el, k in spec:
        L = pv.Light(position=tuple(f + _unit(az, el) * R * 4), focal_point=tuple(f), intensity=k,
                     light_type="scene light")
        out.append((name, az, el, k, L))
    return out


def _scene(pv, rows, spin, cam, view_angle, lights, bg, px, shadows, origins=None):
    """Studio plotter: three-point lights, smooth shading split at sharp edges,
    SSAA; returns (plotter, {name: actor} of the spinning parts, {name: actor})."""
    pl = pv.Plotter(off_screen=True, window_size=(px, px), lighting="none", line_smoothing=True)
    pl.set_background(**BACKGROUNDS[bg])
    for *_, L in lights:
        pl.add_light(L.copy() if hasattr(L, "copy") else L)
    actors, every = {}, {}
    for n, m, c, o in rows:
        a = pl.add_mesh(m, color=c, opacity=o, smooth_shading=True, split_sharp_edges=True, feature_angle=35,
                        show_edges=False, ambient=0.22, diffuse=0.80, specular=0.18 if o >= 1 else 0.08,
                        specular_power=25)
        every[n] = a
        if n in spin:
            a.SetOrigin(*(origins or {}).get(n, (0, 0, 0)))
            actors[n] = a
    pl.camera_position = cam
    pl.camera.view_angle = view_angle
    pl.enable_anti_aliasing("ssaa")
    if shadows:
        pl.enable_shadows()
    return pl, actors, every


def visibility(pv, rows, spin, watch, cam, view_angle, lights, bg, px, frames, thresh=60, origins=None):
    """Per-frame visible pixel count of the `watch` parts at px: each frame is
    rendered twice, normally and with the watched actors hidden, and a pixel
    counts only when the summed absolute RGB difference is at least `thresh`
    (60 of 255), so a part washed out behind clear acrylic, the net or a
    highlight does not count as readable. Shadows are
    off here so a hidden part's cast shadow is not counted as the part."""
    pl, actors, every = _scene(pv, rows, spin, cam, view_angle, lights, bg, px, False, origins)
    out = []
    for i in range(frames):
        for a in actors.values():
            a.orientation = (0.0, 0.0, i * 360.0 / frames)
        pl.render()
        on = pl.screenshot(None, return_img=True)[:, :, :3].astype(np.int16)
        for n in watch:
            every[n].SetVisibility(False)
        pl.render()
        off = pl.screenshot(None, return_img=True)[:, :, :3].astype(np.int16)
        for n in watch:
            every[n].SetVisibility(True)
        out.append(int((np.abs(on - off).sum(axis=2) >= thresh).sum()))
    pl.close()
    return out


def animate(A, outdir, frames=72, fps=24, loops=2, px=1600, gif_w=GIF_W, bg="gradient", shadows=True,
            elev=ELEV, watch=WATCH, vis_px=1000, stem="drum_spin", log=print):
    t0 = time.time()
    pv = _pv()
    M = MS.meshes(A)
    spin = set(spinning(A)) & set(M)
    origins = {n: A.parts[n].meta.get("spin_origin", (0, 0, 0)) for n in spin}
    rows = [(n, _pv_mesh(pv, *M[n]), A.parts[n].colour, A.parts[n].meta.get("opacity", 1.0)) for n in M]
    allv = np.concatenate([M[n][0] for n in M])
    lo, hi = allv.min(0), allv.max(0)
    # the spinning drum sweeps a full circle: widen the box to its swept radius
    sv = np.concatenate([M[n][0] for n in spin])
    rmax = np.hypot(sv[:, 0], sv[:, 1]).max()
    lo[:2] = np.minimum(lo[:2], -rmax)
    hi[:2] = np.maximum(hi[:2], rmax)
    rot = [n for n in spin if A.parts[n].module == "rotor"]
    zr = (min(A.bbox(n)[0][2] for n in rot), max(A.bbox(n)[1][2] for n in rot))
    # aim halfway between the scene's mid-height and the drum's, on the spin axis
    focus = np.array((0.0, 0.0, 0.5 * (0.5 * (lo[2] + hi[2]) + 0.5 * (zr[0] + zr[1]))))
    view_angle = 26.0
    still = np.concatenate([M[n][0] for n in M if n not in spin])
    swept = [sv @ np.array([[np.cos(a), np.sin(a), 0], [-np.sin(a), np.cos(a), 0], [0, 0, 1]])
             for a in np.radians(np.arange(0, 360, 30))]
    pts = np.concatenate([still] + swept)
    cam = _frame_camera(pts, focus, _unit(AZIM, elev), view_angle)
    t_mesh = time.time() - t0
    log(f"  animate: {len(rows)} meshes, {len(spin)} spinning, elev {elev} deg az {AZIM} deg, "
        f"focus z {focus[2]:.1f} ({t_mesh:.1f} s)")

    lights = _lights(pv, focus, float(np.linalg.norm(hi - lo)))
    os.makedirs(outdir, exist_ok=True)
    step = 360.0 / frames
    vis = None
    if watch:
        t1 = time.time()
        vis = visibility(pv, rows, spin, set(watch), cam, view_angle, lights, bg, vis_px, frames, origins=origins)
        log(f"  animate: visibility of {'+'.join(watch)} at {vis_px} px: min {min(vis)} median "
            f"{int(np.median(vis))} max {max(vis)} px ({time.time() - t1:.1f} s)")
    pl, actors, _ = _scene(pv, rows, spin, cam, view_angle, lights, bg, px, shadows, origins)
    imgs = []
    t1 = time.time()
    for i in range(frames):
        ang = i * step
        for a in actors.values():
            a.orientation = (0.0, 0.0, ang)
        pl.render()
        img = pl.screenshot(None, return_img=True)
        imgs.append(np.ascontiguousarray(img[:, :, :3]))
        if i == 0:
            log(f"  animate: first frame {time.time() - t1:.2f} s")
    pl.close()
    t_render = time.time() - t1
    log(f"  animate: {frames} frames in {t_render:.1f} s ({t_render / frames:.2f} s/frame)")

    h, w = imgs[0].shape[:2]
    out = dict(frames=frames, fps=fps, loops=loops, resolution=[int(w), int(h)], elevation_deg=elev,
               azimuth_deg=AZIM, view_angle_deg=view_angle, background=bg, shadows=shadows,
               lights=[dict(name=nm, azimuth_deg=az, elevation_deg=el, intensity=k) for nm, az, el, k, _ in lights],
               spinning=sorted(spin), spinning_count=len(spin), render_seconds=round(t_render, 2),
               mesh_seconds=round(t_mesh, 2))
    exe, src = _encoder()
    raw = ["-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-"]

    def pipe(args, reps):
        p = subprocess.Popen([exe, "-y", "-loglevel", "error", *raw, *args], stdin=subprocess.PIPE)
        for _ in range(reps):
            for im in imgs:
                p.stdin.write(im.tobytes())
        p.stdin.close()
        if p.wait() != 0:
            raise RuntimeError(f"encoder exited {p.returncode}: {args[-1]}")
        return args[-1]

    if exe:
        gif = pipe(["-vf", f"scale={gif_w}:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors=256:"
                    "stats_mode=full[p];[b][p]paletteuse=dither=sierra2_4a", "-loop", "0",
                    os.path.join(outdir, f"{stem}.gif")], 1)
        mp4 = pipe(["-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", "-preset", "slow",
                    "-movflags", "+faststart", os.path.join(outdir, f"{stem}.mp4")], loops)
        out.update(encoder=f"via {src} ({exe})", gif=gif, video=mp4, video_bytes=os.path.getsize(mp4),
                   duration_s=round(frames * loops / fps, 3))
    else:
        import imageio.v2 as iio
        from PIL import Image
        gif = os.path.join(outdir, f"{stem}.gif")
        small = [np.asarray(Image.fromarray(im).resize((gif_w, round(h * gif_w / w)), Image.LANCZOS)) for im in imgs]
        iio.mimsave(gif, small, duration=1.0 / fps, loop=0)
        out.update(encoder="imageio GIF only (no H.264 encoder found, no MP4)", gif=gif)
    if vis is not None:
        out["visibility"] = dict(parts=list(watch), px=vis_px, threshold="sum |dRGB| >= 60", min=min(vis), median=int(np.median(vis)), max=max(vis),
                                 per_frame=vis)
    out["gif_bytes"] = os.path.getsize(gif)
    out["gif_width"] = gif_w
    out["gif_duration_s"] = round(frames / fps, 3)
    out["sheet"] = contact_sheet(imgs, [0, 90, 180, 270], step, os.path.join(outdir, f"{stem}_sheet.png"))
    out["sheet_bytes"] = os.path.getsize(out["sheet"])
    out["total_seconds"] = round(time.time() - t0, 2)
    return out


def contact_sheet(imgs, angles, step, path, cell=600):
    from PIL import Image, ImageDraw, ImageFont
    pad = 60
    sheet = Image.new("RGB", (cell * len(angles), cell + pad), "white")
    d = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 36)
    except Exception:  # noqa: BLE001
        font = ImageFont.load_default()
    for k, ang in enumerate(angles):
        im = Image.fromarray(imgs[int(round(ang / step)) % len(imgs)]).resize((cell, cell), Image.LANCZOS)
        sheet.paste(im, (k * cell, pad))
        d.text((k * cell + 20, 10), f"rotor {ang} deg", fill="black", font=font)
        if k:
            d.line([(k * cell, 0), (k * cell, cell + pad)], fill=(160, 160, 160), width=2)
    sheet.save(path)
    return path
