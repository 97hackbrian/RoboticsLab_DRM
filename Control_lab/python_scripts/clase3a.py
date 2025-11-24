import math
import random
import pygame
import numpy as np
import matplotlib.pyplot as plt  # <-- agregado
import time                       # <-- agregado

# =============== Config ===============
WIDTH, HEIGHT = 1000, 700
FPS = 60

# Robot params (unicycle model)
MAX_V = 220.0         # px/s (linear speed saturation)
MAX_W = 6.0           # rad/s (angular speed saturation)
W_NOISE_STD = 0.05    # rad/s noise amplitude when noise enabled

# Controller gains (start values)
Kp, Ki, Kd = 1.20, 0.20, 0.10

# Anti-windup
TAU_AW = 0.20         # s (smaller -> stronger antiwindup)

# Distance controller (simple P gain on v)
Kv = 1.2              # speed toward target
ARRIVE_THRESH = 8.0   # px

# Colors
WHITE=(255,255,255); BLACK=(0,0,0); RED=(230,60,60); GREEN=(60,200,60)
BLUE=(60,140,255); ORANGE=(255,160,60); GREY=(210,210,210); PURPLE=(160,80,255)

# =============== Helpers ===============
def angle_wrap(a):
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:  a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def draw_robot(surf, x, y, th, color=BLUE):
    """Draw a triangle robot pointing at heading th."""
    L = 20
    W = 12
    pts = []
    pts.append((x + L*math.cos(th),     y + L*math.sin(th)))         # nose
    pts.append((x + -L*0.6*math.cos(th) - W*math.sin(th),
                y + -L*0.6*math.sin(th) + W*math.cos(th)))          # left
    pts.append((x + -L*0.6*math.cos(th) + W*math.sin(th),
                y + -L*0.6*math.sin(th) - W*math.cos(th)))          # right
    pygame.draw.polygon(surf, color, pts)
    pygame.draw.circle(surf, BLACK, (int(x), int(y)), 2)

def text(surf, s, xy, color=BLACK, size=18):
    f = pygame.font.SysFont("Consolas", size)
    surf.blit(f.render(s, True, color), xy)

# =============== Main ===============
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Interactive PID Lab (unicycle robot) — click to set goals")
    clock = pygame.time.Clock()

    # State: position (px) and heading (rad)
    x, y, th = WIDTH*0.2, HEIGHT*0.75, -math.pi/4
    trail = []
    paused = False
    noise_on = False

    # Target (goal) — fixed setpoint requested
    goal = np.array([949.0, 648.0], dtype=float)

    # PID states (heading controller)
    integ = 0.0
    prev_e = 0.0

    # For disturbance pulse
    disturb_timer = 0.0
    disturb_w = 0.0

    # Real-time plotting (matplotlib, non-blocking)
    time_hist = []
    x_hist = []
    y_hist = []
    sim_time = 0.0
    plot_dt = 0.05      # update plot every 0.05s
    plot_timer = 0.0
    plot_window = 30.0  # seconds of history shown

    plt.ion()
    fig, ax = plt.subplots()
    line_x, = ax.plot([], [], label='x(t)', color='tab:blue')
    line_y, = ax.plot([], [], label='y(t)', color='tab:orange')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('position (px)')
    ax.grid(True)
    ax.legend()
    ax.set_xlim(0, plot_window)
    ax.set_ylim(0, max(WIDTH, HEIGHT))

    # --- Per-setpoint plot state (starts on mouse click) ---
    click_plot_active = False
    click_fig = None
    click_ax = None
    click_line_x = None
    click_line_y = None
    click_time_hist = []
    click_x_hist = []
    click_y_hist = []
    click_start_sim_time = 0.0
    click_goal = None

    # --- saturation / anti-windup mode ---
    sat_modes = ['backcalc', 'integrator_clamp', 'conditional', 'smooth_tanh']
    sat_mode = 0  # index into sat_modes, press M to cycle

    global Kp, Ki, Kd  # so we can tweak with keys

    running = True
    while running:
        dt_ms = clock.tick(FPS)
        dt = dt_ms / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                # Start a per-setpoint plot for the FIXED goal (click will NOT change goal)
                click_plot_active = True
                click_time_hist = []
                click_x_hist = []
                click_y_hist = []
                click_start_sim_time = sim_time
                click_goal = goal.copy()
                try:
                    click_fig, click_ax = plt.subplots()
                    click_line_x, = click_ax.plot([], [], label='x(t)', color='tab:blue')
                    click_line_y, = click_ax.plot([], [], label='y(t)', color='tab:orange')
                    click_ax.set_xlabel('time (s)')
                    click_ax.set_ylabel('position (px)')
                    click_ax.set_title(f"Response to FIXED setpoint ({int(goal[0])}, {int(goal[1])})  Kp={Kp:.2f} Ki={Ki:.2f} Kd={Kd:.2f}")
                    click_ax.grid(True)
                    click_ax.legend()
                except Exception:
                    click_plot_active = False
                    click_fig = None
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    # reset pose & controller and start per-setpoint plotting for current goal
                    x, y, th = WIDTH*0.2, HEIGHT*0.75, -math.pi/4
                    integ = 0.0; prev_e = 0.0; trail.clear()
                    # start a per-setpoint plot (like mouse click) so you can see response from reset
                    click_plot_active = True
                    click_time_hist = []
                    click_x_hist = []
                    click_y_hist = []
                    click_start_sim_time = sim_time
                    click_goal = goal.copy()
                    try:
                        click_fig, click_ax = plt.subplots()
                        click_line_x, = click_ax.plot([], [], label='x(t)', color='tab:blue')
                        click_line_y, = click_ax.plot([], [], label='y(t)', color='tab:orange')
                        click_ax.set_xlabel('time (s)')
                        click_ax.set_ylabel('position (px)')
                        click_ax.set_title(f"Response to setpoint ({int(goal[0])}, {int(goal[1])})")
                        click_ax.grid(True)
                        click_ax.legend()
                    except Exception:
                        click_plot_active = False
                        click_fig = None
                elif event.key == pygame.K_n:
                    noise_on = not noise_on
                elif event.key == pygame.K_d:
                    # short disturbance pulse on angular velocity
                    disturb_timer = 0.25
                    disturb_w = random.choice([-1.8, 1.8])
                elif event.key == pygame.K_m:
                    # cycle saturation/anti-windup mode
                    sat_mode = (sat_mode + 1) % len(sat_modes)

        # Gain tuning via keys
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:    Kp += 0.05
        if keys[pygame.K_DOWN]:  Kp = max(0.0, Kp - 0.05)
        if keys[pygame.K_RIGHT]: Ki += 0.05
        if keys[pygame.K_LEFT]:  Ki = max(0.0, Ki - 0.05)
        if keys[pygame.K_KP_1]:   Kd += 0.01
        if keys[pygame.K_KP_0]: Kd = max(0.0, Kd - 0.01)

        # Physics update
        if not paused:
            # Vector to goal
            dx, dy = goal[0] - x, goal[1] - y
            dist = math.hypot(dx, dy)
            th_goal = math.atan2(dy, dx)

            # Heading error for PID (wrap)
            e = angle_wrap(th_goal - th)

            # PID (angular velocity command)
            # We'll implement multiple saturation / anti-windup strategies selectable by sat_mode
            deriv = (e - prev_e) / dt if dt > 0 else 0.0

            if sat_modes[sat_mode] == 'backcalc':
                # existing back-calculation anti-windup
                integ += e * dt
                u = Kp * e + Ki * integ + Kd * deriv
                u_sat = max(-MAX_W, min(MAX_W, u))
                du_aw = (u_sat - u) / max(1e-6, TAU_AW)
                integ += du_aw * dt

            elif sat_modes[sat_mode] == 'integrator_clamp':
                # clamp integrator so Ki*integ doesn't force beyond actuator limits
                integ += e * dt
                u = Kp * e + Ki * integ + Kd * deriv
                u_sat = max(-MAX_W, min(MAX_W, u))
                if abs(u - u_sat) > 1e-12 and abs(Ki) > 1e-9:
                    # set integ so Ki*integ = u_sat - Kp*e - Kd*deriv
                    integ = (u_sat - Kp * e - Kd * deriv) / Ki
                    u = Kp * e + Ki * integ + Kd * deriv
                # u_sat already computed

            elif sat_modes[sat_mode] == 'conditional':
                # only integrate when actuator not saturated OR integrator helps reduce saturation
                u_unsat = Kp * e + Ki * integ + Kd * deriv
                u_sat = max(-MAX_W, min(MAX_W, u_unsat))
                # if not saturated -> integrate; if saturated, only integrate when u_unsat reduces saturation (sign check)
                if abs(u_unsat) < MAX_W or (u_unsat * e > 0):
                    integ += e * dt
                u = Kp * e + Ki * integ + Kd * deriv
                u_sat = max(-MAX_W, min(MAX_W, u))

            elif sat_modes[sat_mode] == 'smooth_tanh':
                # smooth saturation using tanh (no hard clipping), reduces windup naturally
                integ += e * dt
                u = Kp * e + Ki * integ + Kd * deriv
                # smooth saturator — preserves differentiability
                u_sat = MAX_W * math.tanh(u / MAX_W)

            else:
                # fallback to simple clipping
                integ += e * dt
                u = Kp * e + Ki * integ + Kd * deriv
                u_sat = max(-MAX_W, min(MAX_W, u))

            # Optional sensor noise on omega
            w_noise = np.random.normal(0.0, W_NOISE_STD) if noise_on else 0.0

            # Distance → linear speed (simple P with saturation; stop near goal)
            v = Kv * dist
            if dist < ARRIVE_THRESH: v = 0.0
            v = max(-MAX_V, min(MAX_V, v))

            # Disturbance pulse
            w_dist = 0.0
            if disturb_timer > 0.0:
                w_dist = disturb_w
                disturb_timer -= dt
                if disturb_timer <= 0.0:
                    disturb_w = 0.0

            # Unicycle dynamics
            w_cmd = u_sat + w_noise + w_dist
            x += v * math.cos(th) * dt
            y += v * math.sin(th) * dt
            th = angle_wrap(th + w_cmd * dt)
            prev_e = e

            # Keep trail
            if len(trail) == 0 or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 2.0:
                trail.append((x, y))
            if len(trail) > 2000:
                trail.pop(0)

            # --- update simulation time and histories for plotting ---
            sim_time += dt
            time_hist.append(sim_time)
            x_hist.append(x)
            y_hist.append(y)
            # trim history to reasonable size
            if len(time_hist) > 20000:
                time_hist.pop(0); x_hist.pop(0); y_hist.pop(0)

            # --- also record for active per-setpoint plot ---
            if click_plot_active:
                # use relative time from click for x-axis (easier to read)
                click_time_hist.append(sim_time - click_start_sim_time)
                click_x_hist.append(x)
                click_y_hist.append(y)

                # if arrived, finalize this per-setpoint plot: draw, save and stop updating
                if dist < ARRIVE_THRESH:
                    try:
                        if click_fig is not None:
                            click_ax.set_xlim(0, max(1.0, click_time_hist[-1]))
                            ys_combined = click_x_hist + click_y_hist
                            ymin = min(ys_combined) - 10
                            ymax = max(ys_combined) + 10
                            if ymin == ymax:
                                ymin -= 1; ymax += 1
                            click_ax.set_ylim(ymin, ymax)
                            click_line_x.set_data(click_time_hist, click_x_hist)
                            click_line_y.set_data(click_time_hist, click_y_hist)
                            click_fig.canvas.draw_idle()
                            click_fig.canvas.flush_events()
                            # save snapshot (include current gains in filename)
                            fname = f"response_setpoint_{int(click_goal[0])}_{int(click_goal[1])}_Kp{int(Kp*100)}_Ki{int(Ki*100)}_Kd{int(Kd*100)}_{int(time.time())}.png"
                            click_fig.savefig(fname)
                    except Exception:
                        pass
                    # stop per-setpoint logging (figure remains shown)
                    click_plot_active = False

        # ===== Render =====
        screen.fill(WHITE)
        # Grid
        for gx in range(0, WIDTH, 50):
            pygame.draw.line(screen, GREY, (gx, 0), (gx, HEIGHT), 1)
        for gy in range(0, HEIGHT, 50):
            pygame.draw.line(screen, GREY, (0, gy), (WIDTH, gy), 1)

        # Trail
        if len(trail) > 1:
            pygame.draw.lines(screen, ORANGE, False, [(int(px), int(py)) for (px,py) in trail], 2)

        # Goal
        pygame.draw.circle(screen, GREEN, (int(goal[0]), int(goal[1])), 8)
        pygame.draw.circle(screen, GREEN, (int(goal[0]), int(goal[1])), int(ARRIVE_THRESH), 1)

        # Robot
        draw_robot(screen, x, y, th, BLUE)

        # HUD
        hud_y = 10
        text(screen, "Click = set goal | ↑/↓ Kp | ←/→ Ki | PgUp/PgDn Kd | N noise | D disturb | R reset | Space pause | M cycle sat", (10, hud_y)); hud_y += 24
        text(screen, f"Kp={Kp:.2f}  Ki={Ki:.2f}  Kd={Kd:.2f}   antiwindup tau={TAU_AW:.2f}s   noise={'ON' if noise_on else 'OFF'}   paused={'YES' if paused else 'NO'}   sat={sat_modes[sat_mode]}", (10, hud_y)); hud_y += 24
        text(screen, f"pos=({x:6.1f},{y:6.1f})  th={th:+.2f} rad   dist_to_goal={math.hypot(goal[0]-x, goal[1]-y):.1f}px   v_sat={MAX_V:.0f}px/s  w_sat={MAX_W:.2f}rad/s", (10, hud_y)); hud_y += 24

        # Actuator bar (|u| saturation)
        bar_x, bar_y, bar_w, bar_h = 10, hud_y, 300, 16
        pygame.draw.rect(screen, BLACK, (bar_x-1, bar_y-1, bar_w+2, bar_h+2), 1)
        # Map w_cmd to bar
        # We recompute u (noisy/disturbed) magnitude for visualization
        # Just reuse prev_e/integ/deriv? Simpler: show last u_sat magnitude
        # We kept u_sat; show proportion:
        prop = abs(u_sat) / MAX_W
        pygame.draw.rect(screen, PURPLE, (bar_x, bar_y, int(bar_w * prop), bar_h))
        text(screen, f"|omega_cmd| / max = {prop*100:4.0f}%", (bar_x + bar_w + 10, bar_y-2))
        hud_y += 26

        pygame.display.flip()

        # Update matplotlib plot periodically (non-blocking)
        try:
            plot_timer += dt
            if plot_timer >= plot_dt:
                plot_timer = 0.0
                if len(time_hist) > 0:
                    xmin = max(0.0, sim_time - plot_window)
                    ax.set_xlim(xmin, max(plot_window, sim_time))
                    # auto-adjust y limits with margin
                    ys_combined = x_hist + y_hist
                    ymin = min(ys_combined) - 10
                    ymax = max(ys_combined) + 10
                    if ymin == ymax:
                        ymin -= 1; ymax += 1
                    ax.set_ylim(ymin, ymax)
                    line_x.set_data(time_hist, x_hist)
                    line_y.set_data(time_hist, y_hist)
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()

                # also refresh active per-setpoint figure while running toward it
                if click_plot_active and click_fig is not None and len(click_time_hist) > 0:
                    try:
                        click_ax.set_xlim(0, max(1.0, click_time_hist[-1]))
                        ys_combined = click_x_hist + click_y_hist
                        ymin = min(ys_combined) - 10
                        ymax = max(ys_combined) + 10
                        if ymin == ymax:
                            ymin -= 1; ymax += 1
                        click_ax.set_ylim(ymin, ymax)
                        click_line_x.set_data(click_time_hist, click_x_hist)
                        click_line_y.set_data(click_time_hist, click_y_hist)
                        click_fig.canvas.draw_idle()
                        click_fig.canvas.flush_events()
                    except Exception:
                        pass
        except Exception:
            # ignore plotting errors so simulator keeps running
            pass

    pygame.quit()
    try:
        plt.ioff()
        plt.show(block=False)
    except Exception:
        pass

if __name__ == "__main__":
    main()
