

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.awt.RenderingHints;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSpinner;
import javax.swing.SpinnerNumberModel;
import javax.swing.SwingUtilities;
import javax.swing.border.EmptyBorder;

import engine.*;

public class Main {

    /**
     * Starts the program.
     *
     * SwingUtilities.invokeLater makes sure the GUI is created on Swing's event
     * dispatch thread, which is the correct thread for building and updating Swing
     * components.
     */
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            PhysicsEngine physicsEngine = new PhysicsEngine();
            GUI gui = new GUI(physicsEngine);

            JFrame frame = new JFrame("2D Physics Sandbox");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setLayout(new BorderLayout());

            // Controls go on the left, simulation view goes in the center.
            frame.add(createControlPanel(gui, physicsEngine), BorderLayout.WEST);
            frame.add(gui, BorderLayout.CENTER);

            frame.setSize(1100, 700);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);

            // Gives the GUI keyboard focus so key controls work immediately.
            gui.requestFocusInWindow();

            /*
             * Physics runs on its own thread instead of inside the paint method.
             *
             * This keeps the simulation update rate separate from the rendering rate.
             * The physics can run at a stable fixed timestep while the screen repaints
             * whenever Swing is ready.
             */
            Thread physicsThread = new Thread(() -> {
                final double fixedDt = 1.0 / 240.0;

                long previous = System.nanoTime();
                double accumulator = 0.0;

                while (true) {
                    long now = System.nanoTime();

                    // Convert elapsed nanoseconds into seconds.
                    double frameTime = (now - previous) / 1_000_000_000.0;
                    previous = now;

                    /*
                     * Prevents the simulation from exploding after a lag spike.
                     *
                     * If the app freezes for a moment, frameTime could become huge.
                     * Clamping it avoids running a massive number of physics steps at once.
                     */
                    frameTime = Math.min(frameTime, 0.25);
                    accumulator += frameTime;

                    /*
                     * Fixed timestep loop.
                     *
                     * Instead of updating physics using whatever random frameTime happens,
                     * the engine updates in small consistent chunks. This makes collisions,
                     * movement, and forces more stable.
                     */
                    while (accumulator >= fixedDt) {
                        physicsEngine.update(fixedDt);
                        accumulator -= fixedDt;
                    }

                    /*
                     * interpolationAlpha tells rendering how far the current frame is
                     * between two physics steps. This can be used for smoother visuals.
                     */
                    physicsEngine.setInterpolationAlpha(accumulator / fixedDt);

                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException ex) {
                        break;
                    }
                }
            });

            // Daemon means this thread will not keep the program alive after closing.
            physicsThread.setDaemon(true);
            physicsThread.start();

            // Repaints the GUI at roughly 144 FPS.
            new javax.swing.Timer(1000 / 144, e -> gui.repaint()).start();
        });
    }

    /**
     * Builds the left-side control panel.
     *
     * This panel lets the user create objects, choose their shape/color/physics
     * settings, delete objects, create ropes, clear ropes, and reset the camera.
     */
    private static JPanel createControlPanel(GUI gui, PhysicsEngine physicsEngine) {
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        panel.setBorder(new EmptyBorder(10, 10, 10, 10));
        panel.setPreferredSize(new Dimension(260, 0));

        JLabel title = new JLabel("Physics Sandbox");
        title.setAlignmentX(JLabel.LEFT_ALIGNMENT);
        panel.add(title);
        panel.add(Box.createVerticalStrut(10));

        JComboBox<String> shapePicker = new JComboBox<>(new String[] {
                "Circle", "Box", "Triangle", "Static Platform"
        });

        JComboBox<String> colorPicker = new JComboBox<>(new String[] {
                "White", "Red", "Orange", "Cyan", "Green", "Pink", "Yellow", "Gray"
        });
        colorPicker.setSelectedItem("Cyan");

        // These spinners let the user tune object properties before spawning.
        JSpinner sizeX = spinner(0.70, 0.05, 10000.0, 0.05);
        JSpinner sizeY = spinner(0.70, 0.05, 10000.0, 0.05);
        JSpinner velocityX = spinner(0.0, -10000.0, 10000.0, 0.5);
        JSpinner velocityY = spinner(0.0, -10000.0, 10000.0, 0.5);
        JSpinner density = spinner(2.5, 0.1, 100000.0, 0.1);
        JSpinner restitution = spinner(0.5, 0.0, 1.0, 0.05);

        // These spinners tune world-wide settings while the simulation is running.
        JSpinner friction = spinner(physicsEngine.getGlobalFriction(), 0.0, 1000.0, 0.05);
        JSpinner gravity = spinner(physicsEngine.getGravityStrength(), 0.0, 1000.0, 0.1);
        JSpinner airResistance = spinner(physicsEngine.getAirResistanceMultiplier(), 0.0, 1000.0, 0.1);

        JCheckBox infiniteMass = new JCheckBox("Static / infinite mass");

        panel.add(row("Shape", shapePicker));
        panel.add(row("Color", colorPicker));
        panel.add(row("Size X / Radius", sizeX));
        panel.add(row("Size Y", sizeY));
        panel.add(row("Velocity X", velocityX));
        panel.add(row("Velocity Y", velocityY));
        panel.add(row("Density", density));
        panel.add(row("Restitution", restitution));
        panel.add(infiniteMass);
        panel.add(Box.createVerticalStrut(8));

        JLabel worldSettings = new JLabel("World Settings");
        worldSettings.setAlignmentX(JLabel.LEFT_ALIGNMENT);
        panel.add(worldSettings);
        panel.add(row("Friction", friction));
        panel.add(row("Gravity", gravity));
        panel.add(row("Air resistance", airResistance));
        panel.add(Box.createVerticalStrut(8));

        friction.addChangeListener(e -> {
            physicsEngine.setGlobalFriction(getDouble(friction));
            gui.requestFocusInWindow();
        });

        gravity.addChangeListener(e -> {
            physicsEngine.setGravityStrength(getDouble(gravity));
            gui.requestFocusInWindow();
        });

        airResistance.addChangeListener(e -> {
            physicsEngine.setAirResistanceMultiplier(getDouble(airResistance));
            gui.requestFocusInWindow();
        });

        JButton createButton = new JButton("Create at Mouse");
        createButton.setAlignmentX(JButton.LEFT_ALIGNMENT);

        /*
         * Creates an object at the current mouse world position using the values
         * selected in the control panel.
         */
        createButton.addActionListener(e -> {
            gui.createObject(
                    (String) shapePicker.getSelectedItem(),
                    getDouble(sizeX),
                    getDouble(sizeY),
                    getDouble(velocityX),
                    getDouble(velocityY),
                    getDouble(density),
                    getDouble(restitution),
                    toColor((String) colorPicker.getSelectedItem()),
                    infiniteMass.isSelected());

            gui.requestFocusInWindow();
        });

        panel.add(createButton);

        JPanel quickButtons = new JPanel(new GridLayout(0, 2, 5, 5));
        quickButtons.setBorder(BorderFactory.createEmptyBorder(8, 0, 8, 0));
        quickButtons.setMaximumSize(new Dimension(240, 120));

        /*
         * Quick buttons reuse the main create button.
         *
         * They simply change the selected shape/options, then call doClick()
         * so creation logic stays in one place.
         */
        JButton circleButton = new JButton("Circle");
        circleButton.addActionListener(e -> {
            shapePicker.setSelectedItem("Circle");
            createButton.doClick();
        });

        JButton boxButton = new JButton("Box");
        boxButton.addActionListener(e -> {
            shapePicker.setSelectedItem("Box");
            createButton.doClick();
        });

        JButton triangleButton = new JButton("Triangle");
        triangleButton.addActionListener(e -> {
            shapePicker.setSelectedItem("Triangle");
            createButton.doClick();
        });

        JButton platformButton = new JButton("Platform");
        platformButton.addActionListener(e -> {
            shapePicker.setSelectedItem("Static Platform");
            infiniteMass.setSelected(true);
            sizeX.setValue(2.0);
            sizeY.setValue(0.25);
            colorPicker.setSelectedItem("Gray");
            createButton.doClick();
        });

        quickButtons.add(circleButton);
        quickButtons.add(boxButton);
        quickButtons.add(triangleButton);
        quickButtons.add(platformButton);
        panel.add(quickButtons);

        JButton deleteButton = new JButton("Delete Near Mouse");
        deleteButton.setAlignmentX(JButton.LEFT_ALIGNMENT);
        deleteButton.addActionListener(e -> {
            physicsEngine.removeNearestEntity(gui.getCurrentWorldPoint(), 2.0);
            gui.requestFocusInWindow();
        });
        panel.add(deleteButton);

        JButton ropeButton = new JButton("Bind Rope (B)");
        ropeButton.setAlignmentX(JButton.LEFT_ALIGNMENT);
        ropeButton.addActionListener(e -> {
            physicsEngine.bindRope(gui.getCurrentWorldPoint());
            gui.requestFocusInWindow();
        });
        panel.add(ropeButton);

        JButton unropeButton = new JButton("Clear Ropes (U)");
        unropeButton.setAlignmentX(JButton.LEFT_ALIGNMENT);
        unropeButton.addActionListener(e -> {
            physicsEngine.unrope();
            gui.requestFocusInWindow();
        });
        panel.add(unropeButton);

        JButton resetCameraButton = new JButton("Reset Camera");
        resetCameraButton.setAlignmentX(JButton.LEFT_ALIGNMENT);
        resetCameraButton.addActionListener(e -> {
            gui.resetCamera();
            gui.requestFocusInWindow();
        });
        panel.add(resetCameraButton);

        panel.add(Box.createVerticalStrut(10));

        JLabel help = new JLabel(
                "<html>Left drag: grab object<br>Right drag: move camera<br>Delete: remove object<br>B: rope, U: clear ropes<br>Air 1.0 = 0.25 linear, 0.10 angular</html>");
        help.setAlignmentX(JLabel.LEFT_ALIGNMENT);
        panel.add(help);

        return panel;
    }

    /**
     * Creates a labeled row for the control panel.
     */
    private static JPanel row(String label, java.awt.Component component) {
        JPanel row = new JPanel(new BorderLayout(5, 0));
        row.setMaximumSize(new Dimension(240, 30));
        row.add(new JLabel(label), BorderLayout.WEST);
        row.add(component, BorderLayout.CENTER);
        return row;
    }

    /**
     * Creates a numeric spinner with shared setup.
     */
    private static JSpinner spinner(double value, double min, double max, double step) {
        return new JSpinner(new SpinnerNumberModel(value, min, max, step));
    }

    /**
     * Reads a spinner value as a double.
     */
    private static double getDouble(JSpinner spinner) {
        return ((Number) spinner.getValue()).doubleValue();
    }

    /**
     * Converts the selected color name into a real Color object.
     */
    private static Color toColor(String name) {
        if ("Red".equals(name))
            return Color.RED;
        if ("Orange".equals(name))
            return Color.ORANGE;
        if ("Cyan".equals(name))
            return Color.CYAN;
        if ("Green".equals(name))
            return Color.GREEN;
        if ("Pink".equals(name))
            return Color.PINK;
        if ("Yellow".equals(name))
            return Color.YELLOW;
        if ("Gray".equals(name))
            return Color.GRAY;
        return Color.WHITE;
    }
}

class GUI extends JPanel {

    // Pixels per world unit.
    private static final int SCALE = 50;

    // Very small velocities do not get arrows drawn.
    private static final double VELOCITY_ARROW_THRESHOLD = 0.05;

    private final PhysicsEngine physicsEngine;

    // Camera center in world coordinates.
    private double cameraX = 0.0;
    private double cameraY = 3.0;

    // Camera panning state.
    private int lastPanX;
    private int lastPanY;
    private boolean panning = false;

    /**
     * Creates the simulation panel.
     *
     * The GUI handles mouse input, keyboard shortcuts, camera movement, object
     * creation, and drawing the current physics world.
     */
    public GUI(PhysicsEngine physicsEngine) {
        this.physicsEngine = physicsEngine;

        setBackground(Color.BLACK);
        setFocusable(true);

        /*
         * One MouseAdapter handles clicking, dragging, panning, and mouse movement.
         *
         * Left mouse is used for grabbing physics objects.
         * Right mouse is used for moving the camera.
         */
        MouseAdapter mouseAdapter = new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                requestFocusInWindow();

                if (SwingUtilities.isRightMouseButton(e)) {
                    panning = true;
                    lastPanX = e.getX();
                    lastPanY = e.getY();
                    return;
                }

                if (SwingUtilities.isLeftMouseButton(e)) {
                    updateMouseState(e, true);
                }
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                if (SwingUtilities.isRightMouseButton(e)) {
                    panning = false;
                    return;
                }

                if (SwingUtilities.isLeftMouseButton(e)) {
                    updateMouseState(e, false);
                }
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                if (panning) {
                    int dx = e.getX() - lastPanX;
                    int dy = e.getY() - lastPanY;

                    /*
                     * Dragging right should move the camera view right, so the camera
                     * position moves in the opposite direction of the screen drag.
                     *
                     * Y is flipped because screen coordinates go down as y increases,
                     * while world coordinates go up as y increases.
                     */
                    cameraX -= dx / (double) SCALE;
                    cameraY += dy / (double) SCALE;

                    lastPanX = e.getX();
                    lastPanY = e.getY();

                    updateMousePosition(e);
                    repaint();
                    return;
                }

                updateMousePosition(e);
            }

            @Override
            public void mouseMoved(MouseEvent e) {
                updateMousePosition(e);
            }
        };

        addMouseListener(mouseAdapter);
        addMouseMotionListener(mouseAdapter);

        /*
         * Keyboard shortcuts for common sandbox actions.
         */
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                if (e.getKeyCode() == KeyEvent.VK_B) {
                    physicsEngine.bindRope(getCurrentWorldPoint());
                }

                if (e.getKeyCode() == KeyEvent.VK_U) { // clears all ropes
                    physicsEngine.unrope();
                }

                if (e.getKeyCode() == KeyEvent.VK_DELETE || e.getKeyCode() == KeyEvent.VK_BACK_SPACE) { // removes the nearest entity
                    physicsEngine.removeNearestEntity(getCurrentWorldPoint(), 2.0);
                }

                if (e.getKeyCode() == KeyEvent.VK_SPACE) {
                    physicsEngine.togglePause();
                }

                if (e.getKeyCode() == KeyEvent.VK_ESCAPE) {
                    physicsEngine.resetSimulation();
                }

                if (e.getKeyCode() == KeyEvent.VK_1) {
                    createObject("Circle", 0.7, 0.7, 0.0, 0.0, 2.5, 0.5, Color.CYAN, false);
                }

                if (e.getKeyCode() == KeyEvent.VK_2) {
                    createObject("Box", 0.7, 0.7, 0.0, 0.0, 2.5, 0.5, Color.ORANGE, false);
                }

                if (e.getKeyCode() == KeyEvent.VK_3) {
                    createObject("Triangle", 0.7, 0.7, 0.0, 0.0, 2.5, 0.5, Color.PINK, false);
                }
                if (e.getKeyCode() == KeyEvent.VK_4) {
                    createObject("Static Platform", 2.0, 0.25, 0.0, 0.0, 2.5, 0.5, Color.GRAY, true);
                }
                if (e.getKeyCode() == KeyEvent.VK_R) {
                    physicsEngine.setMouseMode(PhysicsEngine.MouseMode.RAGDOLL);
                }

                if (e.getKeyCode() == KeyEvent.VK_T) {
                    physicsEngine.setMouseMode(PhysicsEngine.MouseMode.STATIC);
                }

                if (e.getKeyCode() == KeyEvent.VK_Q) {
                    physicsEngine.rotateGrabbedBody(-Math.toRadians(5.0));
                }

                if (e.getKeyCode() == KeyEvent.VK_E) {
                    physicsEngine.rotateGrabbedBody(Math.toRadians(5.0));
                }
            }
        });
    }

    /**
     * Lightweight render data for drawing a body.
     *
     * This is separate from PhysicsBody so rendering can use only the information
     * it needs instead of depending on the full simulation object.
     */
    public static class RenderBody {
        public double x, y;
        public double rotation;
        public Collider collider;
        public Color color;

        public RenderBody(double x, double y, double rotation, Collider collider, Color color) {
            this.x = x;
            this.y = y;
            this.rotation = rotation;
            this.collider = collider;
            this.color = color;
        }
    }

    /**
     * Creates a new object at the current mouse position.
     *
     * The control panel and keyboard shortcuts both call this method, so all object
     * creation from the GUI goes through one place.
     */
    public void createObject(
            String shape,
            double sizeX,
            double sizeY,
            double velocityX,
            double velocityY,
            double density,
            double restitution,
            Color color,
            boolean infiniteMass) {

        Vector2 p = getCurrentWorldPoint();
        Vector2 velocity = new Vector2(velocityX, velocityY);

        if ("Circle".equals(shape)) {
            physicsEngine.spawnCircle(sizeX, p.x, p.y, velocity, density, restitution, color, infiniteMass);
        } else if ("Box".equals(shape)) {
            physicsEngine.spawnRectangle(sizeX, sizeY, p.x, p.y, velocity, density, restitution, color, infiniteMass);
        } else if ("Triangle".equals(shape)) {
            physicsEngine.spawnTriangle(sizeX, sizeY, p.x, p.y, velocity, density, restitution, color, infiniteMass);
        } else if ("Static Platform".equals(shape)) {
            physicsEngine.spawnRectangle(sizeX, sizeY, p.x, p.y, Vector2.zero(), density, restitution, color, true);
        }
    }

    /**
     * Returns the current mouse point in world coordinates.
     *
     * If the mouse has not entered the panel yet, the camera center is used instead
     * so object creation still has a valid location.
     */
    public Vector2 getCurrentWorldPoint() {
        Vector2 mouse = physicsEngine.getMousePosition();

        if (mouse != null) {
            return mouse;
        }

        return new Vector2(cameraX, cameraY);
    }

    public void resetCamera() {
        cameraX = 0.0;
        cameraY = 3.0;
    }

    /**
     * Updates both mouse position and whether the mouse is currently pressed.
     */
    private void updateMouseState(MouseEvent e, boolean down) {
        physicsEngine.setMousePosition(screenToWorld(e.getX(), e.getY()));
        physicsEngine.setMouseDown(down);
    }

    /**
     * Sends the latest mouse world position to the physics engine.
     */
    private void updateMousePosition(MouseEvent e) {
        physicsEngine.setMousePosition(screenToWorld(e.getX(), e.getY()));
    }

    /**
     * Converts screen coordinates into world coordinates.
     *
     * Screen x/y are pixel values from the panel.
     * World x/y are physics coordinates used by the simulation.
     *
     * The y formula is reversed because screen y increases downward, but world y
     * increases upward.
     */
    private Vector2 screenToWorld(int sx, int sy) {
        double x = cameraX + (sx - getWidth() / 2.0) / SCALE;
        double y = cameraY + (getHeight() / 2.0 - sy) / SCALE;

        return new Vector2(x, y);
    }

    /**
     * Converts a world x-position into a screen x-position.
     */
    private int worldToScreenX(double worldX) {
        return getWidth() / 2 + (int) Math.round((worldX - cameraX) * SCALE);
    }

    /**
     * Converts a world y-position into a screen y-position.
     *
     * The subtraction flips y so positive world y appears upward on screen.
     */
    private int worldToScreenY(double worldY) {
        return getHeight() / 2 - (int) Math.round((worldY - cameraY) * SCALE);
    }

    /**
     * Draws the entire simulation panel.
     *
     * The drawing order matters:
     *
     * 1. Clear the screen.
     * 2. Draw camera info.
     * 3. Draw rope preview if placing a rope.
     * 4. Draw existing ropes.
     * 5. Draw bodies.
     * 6. Draw velocity labels and arrows above bodies.
     */
    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2d = (Graphics2D) g;

        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2d.clearRect(0, 0, getWidth(), getHeight());

        drawCameraInfo(g2d);

        if (physicsEngine.isRopePlacing()) {
            drawFiberRope(
                    g2d,
                    physicsEngine.getRopePointA(),
                    physicsEngine.getMousePosition());
        }

        for (RopeJoint rope : physicsEngine.getRopes()) {
            drawRope(g2d, rope);
        }

        for (BodyEntity entity : physicsEngine.getEntities()) {
            drawEntity(g2d, entity);
        }

        for (BodyEntity entity : physicsEngine.getEntities()) {
            drawVelocityIndicator(g2d, entity);
        }
    }

    private void drawCameraInfo(Graphics2D g2d) {
        g2d.setColor(Color.LIGHT_GRAY);

        g2d.drawString(
                String.format("Camera: (%.2f, %.2f)", cameraX, cameraY),
                10,
                18);

        g2d.drawString(
                "Mouse Mode: " + physicsEngine.getMouseMode(),
                10,
                36);
        g2d.drawString(
                physicsEngine.isPaused() ? "Simulation: PAUSED" : "Simulation: RUNNING",
                10,
                56);

        g2d.drawString(
                String.format(
                        "Friction: %.2f  Gravity: %.2f  Air: %.2f",
                        physicsEngine.getGlobalFriction(),
                        physicsEngine.getGravityStrength(),
                        physicsEngine.getAirResistanceMultiplier()),
                10,
                76);
    }

    private void drawRope(Graphics2D g2d, RopeJoint rope) {
        g2d.setColor(Color.BLACK);
        drawFiberRope(g2d, rope.getWorldAnchorA(), rope.getWorldAnchorB());
    }

    /**
     * Draws a rope with a twisted-fiber look.
     *
     * Instead of drawing one straight line, this draws three sinusoidal strands
     * around the rope direction. Each strand has a different phase, which makes
     * the rope look braided.
     */
    private void drawFiberRope(Graphics2D g2d, Vector2 a, Vector2 b) {
        if (a == null || b == null) {
            return;
        }

        int segments = 80;

        Vector2 delta = b.subtract(a);
        double length = delta.magnitude();

        if (length < 1e-8) {
            return;
        }

        /*
         * dir points along the rope.
         * normal points perpendicular to the rope.
         *
         * The sine wave offset is applied along the normal direction to make each
         * strand curve around the rope line.
         */
        Vector2 dir = delta.multiply(1.0 / length);
        Vector2 normal = new Vector2(-dir.y, dir.x);

        double stretch = Math.min(2.0, length);
        double radius = 0.05;
        double twists = length * 5.0;

        for (int strand = 0; strand < 3; strand++) {
            int[] xs = new int[segments + 1];
            int[] ys = new int[segments + 1];

            // Phase shifts each strand so the three curves do not overlap.
            double phase = strand * (Math.PI * 2.0 / 3.0);

            for (int i = 0; i <= segments; i++) {
                double t = i / (double) segments;

                Vector2 base = a.add(delta.multiply(t));

                double coil = Math.sin(t * Math.PI * 2.0 * twists + phase);
                double compression = 1.0 / stretch;

                Vector2 offset = normal.multiply(coil * radius * compression);
                Vector2 p = base.add(offset);

                xs[i] = worldToScreenX(p.x);
                ys[i] = worldToScreenY(p.y);
            }

            g2d.drawPolyline(xs, ys, segments + 1);
        }
    }

    /**
     * Draws an entity based on its collider shape.
     */
    private void drawEntity(Graphics2D g2d, BodyEntity entity) {
        PhysicsBody body = entity.getBody();
        Collider collider = body.getCollider();

        g2d.setColor(entity.getColor());

        if (collider.getShapeType() == ShapeType.CIRCLE) {
            drawCircle(g2d, body, collider);
        } else if (collider.getShapeType() == ShapeType.RECTANGLE) {
            drawRectangle(g2d, body, collider);
        } else if (collider.getShapeType() == ShapeType.POLYGON) {
            drawPolygon(g2d, body, collider);
        }
    }

    /**
     * Draws a circle using its center position and radius.
     */
    private void drawCircle(Graphics2D g2d, PhysicsBody body, Collider collider) {
        Vector2 position = body.getPosition();

        int sx = worldToScreenX(position.x);
        int sy = worldToScreenY(position.y);

        int r = Math.max(1, (int) Math.round(collider.getRadius() * SCALE));

        g2d.fillOval(sx - r, sy - r, r * 2, r * 2);
    }

    /**
     * Builds a rectangle's local vertices and draws them as a rotated world
     * polygon.
     */
    private void drawRectangle(Graphics2D g2d, PhysicsBody body, Collider collider) {
        double hw = collider.getHalfWidth();
        double hh = collider.getHalfHeight();

        Vector2[] vertices = new Vector2[] {
                new Vector2(-hw, -hh),
                new Vector2(hw, -hh),
                new Vector2(hw, hh),
                new Vector2(-hw, hh)
        };

        drawWorldPolygon(g2d, body, vertices);
    }

    /**
     * Draws a polygon using the vertices stored in its collider.
     */
    private void drawPolygon(Graphics2D g2d, PhysicsBody body, Collider collider) {
        drawWorldPolygon(g2d, body, collider.getVertices());
    }

    /**
     * Converts local shape vertices into screen coordinates and draws the polygon.
     *
     * Local vertices are defined around the body's center. To draw them correctly,
     * each vertex is rotated by the body's current rotation and then moved by the
     * body's world position.
     */
    private void drawWorldPolygon(Graphics2D g2d, PhysicsBody body, Vector2[] localVertices) {
        int n = localVertices.length;
        int[] xs = new int[n];
        int[] ys = new int[n];

        double cos = Math.cos(body.getRotation());
        double sin = Math.sin(body.getRotation());

        for (int i = 0; i < n; i++) {
            Vector2 local = localVertices[i];

            double worldX = body.getPosition().x + local.x * cos - local.y * sin;
            double worldY = body.getPosition().y + local.x * sin + local.y * cos;

            xs[i] = worldToScreenX(worldX);
            ys[i] = worldToScreenY(worldY);
        }

        g2d.fillPolygon(xs, ys, n);
    }

    /**
     * Draws a speed label and velocity arrow for dynamic bodies.
     *
     * Static bodies are skipped because they do not move. Very slow bodies still
     * get a speed label, but no arrow because the arrow would be too small to help.
     */
    private void drawVelocityIndicator(Graphics2D g2d, BodyEntity entity) {
        PhysicsBody body = entity.getBody();

        if (body.getMassType() == MassType.INFINITE) {
            return;
        }

        Vector2 velocity = body.getLinearVelocity();
        double speed = velocity.magnitude();
        Vector2 position = body.getPosition();

        int cx = worldToScreenX(position.x);
        int cy = worldToScreenY(position.y);
        int objectOffset = getObjectScreenOffset(body.getCollider());

        String label = String.format("%.2f m/s", speed);

        FontMetrics metrics = g2d.getFontMetrics();
        int labelX = cx - metrics.stringWidth(label) / 2;
        int labelY = cy + objectOffset + 18;

        g2d.setColor(Color.PINK);
        g2d.drawString(label, labelX, labelY);

        if (speed < VELOCITY_ARROW_THRESHOLD) {
            return;
        }

        Vector2 direction = velocity.normalized();

        // Arrow gets longer as speed increases, but is capped at 90 pixels.
        double arrowLength = Math.min(90.0, 18.0 + speed * 8.0);

        int dx = (int) Math.round(direction.x * arrowLength);

        /*
         * Screen y is flipped compared to world y, so direction.y must be negated.
         */
        int dy = (int) Math.round(-direction.y * arrowLength);

        int startX = cx - dx / 2;
        int startY = cy - dy / 2;
        int endX = cx + dx / 2;
        int endY = cy + dy / 2;

        g2d.setColor(Color.YELLOW);
        g2d.setStroke(new BasicStroke(2.0f));

        g2d.drawLine(startX, startY, endX, endY);
        drawArrowHead(g2d, startX, startY, endX, endY);

        g2d.setStroke(new BasicStroke(1.0f));
    }

    /**
     * Calculates how far below an object the speed label should be drawn.
     *
     * The offset is based on the object's approximate screen height, so the label
     * appears outside the shape instead of inside it.
     */
    private int getObjectScreenOffset(Collider collider) {
        if (collider.getShapeType() == ShapeType.CIRCLE) {
            return (int) Math.round(collider.getRadius() * SCALE);
        }

        if (collider.getShapeType() == ShapeType.RECTANGLE) {
            return (int) Math.round(collider.getHalfHeight() * SCALE);
        }

        Vector2[] vertices = collider.getVertices();
        double maxY = 0.0;

        if (vertices != null) {
            for (Vector2 vertex : vertices) {
                maxY = Math.max(maxY, Math.abs(vertex.y));
            }
        }

        return (int) Math.round(maxY * SCALE);
    }

    /**
     * Draws a simple two-line arrowhead at the end of a velocity arrow.
     */
    private void drawArrowHead(Graphics2D g2d, int startX, int startY, int endX, int endY) {
        double angle = Math.atan2(endY - startY, endX - startX);
        int headLength = 15;

        int x1 = endX - (int) Math.round(headLength * Math.cos(angle - Math.PI / 6.0));
        int y1 = endY - (int) Math.round(headLength * Math.sin(angle - Math.PI / 6.0));

        int x2 = endX - (int) Math.round(headLength * Math.cos(angle + Math.PI / 6.0));
        int y2 = endY - (int) Math.round(headLength * Math.sin(angle + Math.PI / 6.0));

        g2d.drawLine(endX, endY, x1, y1);
        g2d.drawLine(endX, endY, x2, y2);
    }
}