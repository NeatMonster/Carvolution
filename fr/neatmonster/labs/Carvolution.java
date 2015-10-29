package fr.neatmonster.labs;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.RenderingHints;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.font.FontRenderContext;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.RevoluteJointDef;

@SuppressWarnings("serial")
public class Carvolution extends JPanel implements Runnable {
    public class Car {
        private final CarDef  carDef;
        private final Body    chassis;
        private final Body[]  wheels;
        private float         maxPos;
        private float         maxPosY;
        private float         minPosY;
        private int           health;
        private int           frames;
        private boolean       alive;
        private final boolean elite;

        public Car(final CarDef carDef) {
            this.carDef = carDef;

            chassis = createChassis(carDef.chassVertices, carDef.chassDensity);

            wheels = new Body[carDef.wheelCount];
            for (int i = 0; i < carDef.wheelCount; ++i)
                wheels[i] = createWheel(carDef.wheelRadius[i],
                        carDef.wheelDensity[i]);

            float mass = chassis.getMass();
            for (int i = 0; i < carDef.wheelCount; ++i)
                mass += wheels[i].getMass();

            final float[] torque = new float[carDef.wheelCount];
            for (int i = 0; i < carDef.wheelCount; ++i)
                torque[i] = mass * -gravity.y / carDef.wheelRadius[i];

            final RevoluteJointDef jointDef = new RevoluteJointDef();
            for (int i = 0; i < carDef.wheelCount; ++i) {
                final Vec2 randVertex = carDef.chassVertices[carDef.wheelVertex[i]];
                jointDef.localAnchorA.set(randVertex.x, randVertex.y);
                jointDef.localAnchorB.set(0f, 0f);
                jointDef.maxMotorTorque = torque[i];
                jointDef.motorSpeed = -MOTOR_SPEED;
                jointDef.enableMotor = true;
                jointDef.bodyA = chassis;
                jointDef.bodyB = wheels[i];
                world.createJoint(jointDef);
            }

            maxPos = 0f;
            maxPosY = 0f;
            minPosY = 0f;
            health = CAR_HEALTH;
            frames = 0;
            alive = true;
            elite = carDef.elite;
        }

        public boolean checkDeath() {
            final Vec2 position = getPosition();
            if (position.y > maxPosY)
                maxPosY = position.y;
            if (position.y < minPosY)
                minPosY = position.y;
            if (position.x > maxPos + 0.02f) {
                health = CAR_HEALTH;
                maxPos = position.x;
            } else {
                if (position.x > maxPos)
                    maxPos = position.x;
                if (Math.abs(chassis.getLinearVelocity().x) < 0.001f)
                    health -= 5;
                health--;
                if (health <= 0)
                    return true;
            }
            return false;
        }

        private Body createChassis(final Vec2[] vertices, final float density) {
            final BodyDef bodyDef = new BodyDef();
            bodyDef.type = BodyType.DYNAMIC;
            bodyDef.position.set(0f, 4f);

            final Body body = world.createBody(bodyDef);

            createChassisPart(body, vertices[0], vertices[1], density);
            createChassisPart(body, vertices[1], vertices[2], density);
            createChassisPart(body, vertices[2], vertices[3], density);
            createChassisPart(body, vertices[3], vertices[4], density);
            createChassisPart(body, vertices[4], vertices[5], density);
            createChassisPart(body, vertices[5], vertices[6], density);
            createChassisPart(body, vertices[6], vertices[7], density);
            createChassisPart(body, vertices[7], vertices[0], density);

            return body;
        }

        private void createChassisPart(final Body body, final Vec2 vertex1,
                final Vec2 vertex2, final float density) {
            final Vec2[] vertices = new Vec2[3];
            vertices[0] = vertex1;
            vertices[1] = vertex2;
            vertices[2] = new Vec2(0f, 0f);

            final FixtureDef fixDef = new FixtureDef();
            final PolygonShape shape = new PolygonShape();
            fixDef.shape = shape;
            fixDef.density = density;
            fixDef.friction = 10f;
            fixDef.restitution = 0.2f;
            fixDef.filter.groupIndex = -1;
            shape.set(vertices, 3);

            body.createFixture(fixDef);
        }

        private Body createWheel(final float radius, final float density) {
            final BodyDef bodyDef = new BodyDef();
            bodyDef.type = BodyType.DYNAMIC;
            bodyDef.position.set(0f, 0f);

            final Body body = world.createBody(bodyDef);

            final FixtureDef fixDef = new FixtureDef();
            final CircleShape shape = new CircleShape();
            fixDef.shape = shape;
            shape.m_radius = radius;
            fixDef.density = density;
            fixDef.friction = 1f;
            fixDef.restitution = 0.2f;
            fixDef.filter.groupIndex = -1;

            body.createFixture(fixDef);

            return body;
        }

        public Vec2 getPosition() {
            return chassis.getPosition();
        }

        public void kill() {
            final float avgSpeed = maxPos / frames * BOX2D_FPS;
            final float score = maxPos + avgSpeed;
            final Score s = new Score();
            s.carDef = carDef;
            s.index = carDef.index;
            s.score = score;
            scores.add(s);

            world.destroyBody(chassis);
            for (final Body wheel : wheels)
                world.destroyBody(wheel);
            alive = false;

            if (cameraTarget == carDef.index)
                cameraTarget = -1;
        }
    }

    public static class CarDef {
        public int      index;
        private boolean elite;
        public float    chassColor;
        public float    chassDensity;
        public Vec2[]   chassVertices;
        public int      wheelCount;
        public float[]  wheelRadius;
        public float[]  wheelDensity;
        public int[]    wheelVertex;
    }

    public static class Score {
        private CarDef carDef;
        private int    index;
        private float  score;
    }

    private static final int WIDTH = 800;

    private static final int HEIGHT    = 600;
    private static final int BOX2D_FPS = 60;

    private static final int POP_COUNT = 20;

    private static final int   ELITE_COUNT  = 1;
    private static final float MUT_PROBA    = 0.05f;
    private static final float MUT_RANGE    = 1f;
    private static final int   ATTRIB_COUNT = 16;
    private static final int   FLOOR_COUNT  = 200;

    private static final float FLOOR_WIDTH   = 1.5f;
    private static final float FLOOR_HEIGHT  = 0.15f;
    private static final float CHASS_MAXAXIS = 1.1f;

    private static final float CHASS_MINAXIS = 0.1f;
    private static final float CHASS_MINDENS = 30f;
    private static final float CHASS_MAXDENS = 300f;
    private static final float WHEEL_MAXRAD  = 0.5f;

    private static final float WHEEL_MINRAD  = 0.2f;
    private static final float WHEEL_MAXDENS = 100f;
    private static final float WHEEL_MINDENS = 40f;
    private static final int   CAR_HEALTH    = BOX2D_FPS * 10;

    private static final float  MOTOR_SPEED = 20f;
    private static final Random rnd         = new Random();

    private static CarDef createRandomCar() {
        final CarDef carDef = new CarDef();

        carDef.chassDensity = rnd.nextFloat() * CHASS_MAXDENS + CHASS_MINDENS;

        carDef.chassVertices = new Vec2[8];
        carDef.chassVertices[0] = new Vec2(
                rnd.nextFloat() * CHASS_MAXAXIS + CHASS_MINAXIS, 0f);
        carDef.chassVertices[1] = new Vec2(
                rnd.nextFloat() * CHASS_MAXAXIS + CHASS_MINAXIS,
                rnd.nextFloat() * CHASS_MAXAXIS + CHASS_MINAXIS);
        carDef.chassVertices[2] = new Vec2(0f,
                rnd.nextFloat() * CHASS_MAXAXIS + CHASS_MINAXIS);
        carDef.chassVertices[3] = new Vec2(
                -rnd.nextFloat() * CHASS_MAXAXIS - CHASS_MINAXIS,
                rnd.nextFloat() * CHASS_MAXAXIS + CHASS_MINAXIS);
        carDef.chassVertices[4] = new Vec2(
                -rnd.nextFloat() * CHASS_MAXAXIS - CHASS_MINAXIS, 0f);
        carDef.chassVertices[5] = new Vec2(
                -rnd.nextFloat() * CHASS_MAXAXIS - CHASS_MINAXIS,
                -rnd.nextFloat() * CHASS_MAXAXIS - CHASS_MINAXIS);
        carDef.chassVertices[6] = new Vec2(0,
                -rnd.nextFloat() * CHASS_MAXAXIS - CHASS_MINAXIS);
        carDef.chassVertices[7] = new Vec2(
                rnd.nextFloat() * CHASS_MAXAXIS + CHASS_MINAXIS,
                -rnd.nextFloat() * CHASS_MAXAXIS - CHASS_MINAXIS);

        carDef.chassColor = rnd.nextFloat();

        carDef.wheelCount = 2;

        carDef.wheelRadius = new float[carDef.wheelCount];
        carDef.wheelDensity = new float[carDef.wheelCount];
        for (int i = 0; i < carDef.wheelCount; ++i) {
            carDef.wheelRadius[i] = rnd.nextFloat() * WHEEL_MAXRAD
                    + WHEEL_MINRAD;
            carDef.wheelDensity[i] = rnd.nextFloat() * WHEEL_MAXDENS
                    + WHEEL_MINDENS;
        }

        carDef.wheelVertex = new int[carDef.wheelCount];
        final List<Integer> free = new ArrayList<Integer>();
        for (int i = 0; i < 8; ++i)
            free.add(i);
        for (int i = 0; i < carDef.wheelCount; ++i) {
            final int index = rnd.nextInt(free.size());
            carDef.wheelVertex[i] = free.get(index);
            free.remove(index);
        }

        return carDef;
    }

    public static Dimension getStringBounds(final Graphics2D g, final Font font,
            final String text) {
        final FontRenderContext ctx = g.getFontRenderContext();
        final double width = font.getStringBounds(text, ctx).getWidth();
        final double height = font.createGlyphVector(ctx, text)
                .getVisualBounds().getHeight();
        return new Dimension((int) width, (int) height);
    }

    public static void main(final String[] args) {
        final JFrame frame = new JFrame();
        frame.setResizable(false);
        frame.setTitle("Carvolution");
        frame.setPreferredSize(new Dimension(WIDTH, HEIGHT));
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        final Carvolution cars = new Carvolution();
        frame.add(cars);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
        cars.run();
    }

    private final Vec2  gravity = new Vec2(0f, -9.81f);
    private final World world   = new World(gravity);

    private final List<Body> floorTiles  = new ArrayList<Body>();
    private final List<Vec2> floorVerts  = new ArrayList<Vec2>();
    private float            cameraX     = 0f;
    private float            cameraY     = 0f;
    private final float      cameraSpeed = 0.05f;

    private int   cameraTarget = -1;
    private float zoom         = 70f;

    private int generation = 0;
    private int deadCars   = 0;

    private final List<Score> scores    = new ArrayList<Score>();
    private final List<Score> topScores = new ArrayList<Score>();

    private final List<Car>    cars    = new ArrayList<Car>();
    private final List<CarDef> carDefs = new ArrayList<CarDef>();

    private int  leader         = 0;
    private Vec2 leaderPosition = new Vec2(0f, 0f);

    private float distance = 0f;

    private float maxDistance = 0f;

    public Carvolution() {
        createFloor();
        firstGeneration();

        final Point point = new Point();
        addMouseListener(new MouseAdapter() {

            private int getTarget(final Point2D click) {
                final Vec2 p = new Vec2((float) click.getX(),
                        (float) click.getY());

                for (int i = cars.size() - 1; i >= 0; i--) {
                    final Car car = cars.get(i);
                    if (car == null || !car.alive)
                        continue;

                    final Body bc = car.chassis;
                    for (Fixture f = bc
                            .getFixtureList(); f != null; f = f.m_next) {
                        final PolygonShape s = (PolygonShape) f.getShape();
                        if (s.testPoint(new Transform(), bc.getLocalPoint(p)))
                            return car.carDef.index;
                    }

                    for (final Body bw : car.wheels)
                        for (Fixture f = bw
                                .getFixtureList(); f != null; f = f.m_next) {
                            final CircleShape s = (CircleShape) f.getShape();
                            if (s.testPoint(new Transform(),
                                    bw.getLocalPoint(p)))
                                return car.carDef.index;
                        }
                }

                return -2;
            }

            @Override
            public void mouseClicked(final MouseEvent e) {
                if (e.getClickCount() == 2) {
                    cameraTarget = -1;
                    return;
                }
                final AffineTransform at = new AffineTransform();
                at.translate(WIDTH / 2.0 - cameraX * zoom - 200.0,
                        HEIGHT / 2.0 + cameraY * zoom);
                at.scale(zoom, -zoom);
                try {
                    at.invert();
                    final Point2D click = at.transform(
                            new Point2D.Double(e.getX(), e.getY()), null);
                    cameraTarget = getTarget(click);
                } catch (final NoninvertibleTransformException e1) {
                    e1.printStackTrace();
                }
            }

            @Override
            public void mousePressed(final MouseEvent e) {
                point.setLocation(e.getPoint());
            }
        });

        addMouseMotionListener(new MouseAdapter() {

            @Override
            public void mouseDragged(final MouseEvent e) {
                cameraTarget = -2;

                final double deltaX = (e.getX() - point.getX()) / zoom;
                final double deltaY = (e.getY() - point.getY()) / zoom;
                cameraX -= deltaX;
                cameraY += deltaY;
                point.setLocation(e.getPoint());
            }
        });

        addMouseWheelListener(new MouseAdapter() {

            @Override
            public void mouseWheelMoved(final MouseWheelEvent e) {
                if (e.getWheelRotation() < 0)
                    zoom *= Math.pow(0.9f, Math.abs(e.getWheelRotation()));
                else
                    zoom *= Math.pow(1.1f, Math.abs(e.getWheelRotation()));

                if (zoom < 1f)
                    zoom = 1f;
                if (zoom > 200f)
                    zoom = 200f;
            }
        });
    }

    private int chooseParent(final int curParent, final int attrIndex,
            final int point1, final int point2) {
        if (point1 == attrIndex || point2 == attrIndex)
            return 1 - curParent;
        return curParent;
    }

    private void createFloor() {
        floorTiles.clear();

        final Vec2 tileVert = new Vec2(-5f, 0f);
        floorVerts.add(tileVert.clone());

        for (int i = 0; i < FLOOR_COUNT; ++i) {
            final Body tile = createFloorTile(tileVert,
                    (rnd.nextFloat() * 3f - 1.5f) * 1.2f * i / FLOOR_COUNT);
            floorTiles.add(tile);
        }
    }

    private Body createFloorTile(final Vec2 position, final float angle) {
        final BodyDef bodyDef = new BodyDef();
        bodyDef.position.set(position.x, position.y);

        final Body body = world.createBody(bodyDef);

        final FixtureDef fixDef = new FixtureDef();
        final PolygonShape shape = new PolygonShape();
        fixDef.shape = shape;
        fixDef.friction = 0.5f;

        final Vec2[] coords = new Vec2[4];
        coords[0] = new Vec2(0f, 0f);
        coords[1] = new Vec2(0f, -FLOOR_HEIGHT);
        coords[2] = new Vec2(FLOOR_WIDTH, -FLOOR_HEIGHT);
        coords[3] = new Vec2(FLOOR_WIDTH, 0f);

        final Vec2 center = new Vec2(0f, 0f);
        final Vec2[] newCoords = rotateFloorTile(coords, center, angle);
        shape.set(newCoords, newCoords.length);

        body.createFixture(fixDef);

        floorVerts.add(body.getWorldPoint(newCoords[3]));
        position.set(body.getWorldPoint(newCoords[3]));
        return body;
    }

    private void findLeader() {
        float lead = 0f;
        for (int i = 0; i < cars.size(); ++i) {
            if (!cars.get(i).alive)
                continue;

            final Vec2 position = cars.get(i).getPosition();
            if (position.x > lead) {
                leaderPosition = position;
                leader = i;
                lead = position.x;
            }
        }
    }

    private void firstGeneration() {
        for (int i = 0; i < POP_COUNT; ++i) {
            final CarDef carDef = createRandomCar();
            carDef.index = i;
            carDefs.add(carDef);
        }

        generation = 0;
        deadCars = 0;

        leader = 0;
        leaderPosition = new Vec2(0f, 0f);

        materializeGeneration();
    }

    private int getRandomParent() {
        final float r = rnd.nextFloat();
        if (r == 0f)
            return 0;
        return (int) Math.floor(-Math.log(r) * POP_COUNT) % POP_COUNT;
    }

    private CarDef makeChild(final CarDef carDef1, final CarDef carDef2) {
        final CarDef carDef = new CarDef();
        final CarDef[] parents = new CarDef[] { carDef1, carDef2 };

        final int point1 = rnd.nextInt(ATTRIB_COUNT);
        int point2 = point1;
        while (point2 == point1)
            point2 = rnd.nextInt(ATTRIB_COUNT);

        int curParent = 0;

        curParent = chooseParent(curParent, 0, point1, point2);
        carDef.chassColor = parents[curParent].chassColor;

        curParent = chooseParent(curParent, 1, point1, point2);
        carDef.chassDensity = parents[curParent].chassDensity;

        carDef.chassVertices = new Vec2[8];
        curParent = chooseParent(curParent, 2, point1, point2);
        carDef.chassVertices[0] = parents[curParent].chassVertices[0];
        curParent = chooseParent(curParent, 3, point1, point2);
        carDef.chassVertices[1] = parents[curParent].chassVertices[1];
        curParent = chooseParent(curParent, 4, point1, point2);
        carDef.chassVertices[2] = parents[curParent].chassVertices[2];
        curParent = chooseParent(curParent, 5, point1, point2);
        carDef.chassVertices[3] = parents[curParent].chassVertices[3];
        curParent = chooseParent(curParent, 6, point1, point2);
        carDef.chassVertices[4] = parents[curParent].chassVertices[4];
        curParent = chooseParent(curParent, 7, point1, point2);
        carDef.chassVertices[5] = parents[curParent].chassVertices[5];
        curParent = chooseParent(curParent, 8, point1, point2);
        carDef.chassVertices[6] = parents[curParent].chassVertices[6];
        curParent = chooseParent(curParent, 9, point1, point2);
        carDef.chassVertices[7] = parents[curParent].chassVertices[7];

        int wheelParent = 0;
        final boolean sameWheelCount = parents[0].wheelCount == parents[1].wheelCount;
        if (!sameWheelCount)
            wheelParent = rnd.nextInt(2);
        carDef.wheelCount = parents[wheelParent].wheelCount;

        carDef.wheelRadius = new float[carDef.wheelCount];
        for (int i = 0; i < carDef.wheelCount; ++i) {
            if (sameWheelCount)
                curParent = chooseParent(curParent, 10 + i, point1, point2);
            else
                curParent = wheelParent;
            carDef.wheelRadius[i] = parents[curParent].wheelRadius[i];
        }
        carDef.wheelDensity = new float[carDef.wheelCount];
        for (int i = 0; i < carDef.wheelCount; ++i) {
            if (sameWheelCount)
                curParent = chooseParent(curParent, 10 + carDef.wheelCount + i,
                        point1, point2);
            else
                curParent = wheelParent;
            carDef.wheelDensity[i] = parents[curParent].wheelDensity[i];
        }

        carDef.wheelVertex = new int[carDef.wheelCount];
        for (int i = 0; i < carDef.wheelCount; ++i) {
            if (sameWheelCount)
                curParent = chooseParent(curParent,
                        10 + 2 * carDef.wheelCount + i, point1, point2);
            else
                curParent = wheelParent;
            carDef.wheelVertex[i] = parents[curParent].wheelVertex[i];
        }

        return carDef;
    }

    private void materializeGeneration() {
        cars.clear();
        for (int i = 0; i < POP_COUNT; ++i)
            cars.add(new Car(carDefs.get(i)));
    }

    private CarDef mutate(final CarDef carDef) {
        for (int i = 0; i < carDef.wheelCount; ++i)
            if (rnd.nextFloat() < MUT_PROBA)
                carDef.wheelRadius[i] = mutateFloat(carDef.wheelRadius[i],
                        WHEEL_MINRAD, WHEEL_MAXRAD);

        for (int i = 0; i < carDef.wheelCount; ++i)
            if (rnd.nextFloat() < MUT_PROBA)
                carDef.wheelVertex[i] = rnd.nextInt(8);

        for (int i = 0; i < carDef.wheelCount; ++i)
            if (rnd.nextFloat() < MUT_PROBA)
                carDef.wheelDensity[i] = mutateFloat(carDef.wheelDensity[i],
                        WHEEL_MINDENS, WHEEL_MAXDENS);

        if (rnd.nextFloat() < MUT_PROBA)
            carDef.chassDensity = mutateFloat(carDef.chassDensity,
                    CHASS_MINDENS, CHASS_MAXDENS);

        mutateVertex(carDef, 0, 1f, 0f);
        mutateVertex(carDef, 1, 1f, 1f);
        mutateVertex(carDef, 2, 0f, 1f);
        mutateVertex(carDef, 3, -1f, 1f);
        mutateVertex(carDef, 4, -1f, 0f);
        mutateVertex(carDef, 5, -1f, -1f);
        mutateVertex(carDef, 6, 0f, -1f);
        mutateVertex(carDef, 7, 1f, -1f);

        carDef.chassColor += rnd.nextFloat() * 0.1f - 0.05f;
        if (carDef.chassColor < 0f)
            carDef.chassColor += 1f;
        if (carDef.chassColor > 0f)
            carDef.chassColor -= 1f;

        return carDef;
    }

    private float mutateFloat(final float old, final float min,
            final float range) {
        final float span = range * MUT_RANGE;
        float base = old - 0.5f * span;
        if (base < min)
            base = min;
        if (base > min + range - span)
            base = min + range - span;
        return base + span * rnd.nextFloat();
    }

    private void mutateVertex(final CarDef carDef, final int n,
            final float xFact, final float yFact) {
        if (rnd.nextFloat() >= MUT_PROBA)
            return;

        final Vec2 v = carDef.chassVertices[n];
        final float x = xFact
                * mutateFloat(xFact * v.x, CHASS_MINAXIS, CHASS_MAXAXIS);
        final float y = yFact
                * mutateFloat(yFact * v.y, CHASS_MINAXIS, CHASS_MAXAXIS);
        carDef.chassVertices[n] = new Vec2(x, y);
    }

    private void newRound() {
        nextGeneration();
        if (cameraTarget != -2)
            cameraTarget = -1;
    }

    private void nextGeneration() {
        final List<CarDef> newGeneration = new ArrayList<CarDef>();

        Collections.sort(scores, new Comparator<Score>() {

            @Override
            public int compare(final Score o1, final Score o2) {
                return o1.score > o2.score ? -1 : 1;
            }
        });

        final Score s = new Score();
        s.carDef = scores.get(0).carDef;
        s.index = generation;
        s.score = scores.get(0).score;
        topScores.add(s);

        for (int i = 0; i < ELITE_COUNT; ++i) {
            scores.get(i).carDef.elite = true;
            scores.get(i).carDef.index = i;
            newGeneration.add(scores.get(i).carDef);
        }

        for (int i = ELITE_COUNT; i < POP_COUNT; ++i) {
            final int parent1 = getRandomParent();
            int parent2 = parent1;
            while (parent2 == parent1)
                parent2 = getRandomParent();
            CarDef child = makeChild(scores.get(parent1).carDef,
                    scores.get(parent2).carDef);
            child = mutate(child);
            child.elite = false;
            child.index = i;
            newGeneration.add(child);
        }

        generation++;
        deadCars = 0;

        scores.clear();
        carDefs.clear();
        carDefs.addAll(newGeneration);

        leader = 0;
        leaderPosition = new Vec2(0f, 0f);

        materializeGeneration();
    }

    @Override
    public void paint(final Graphics g_) {
        final Graphics2D g = (Graphics2D) g_;
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL,
                RenderingHints.VALUE_STROKE_NORMALIZE);

        synchronized (cars) {
            setCameraPosition();
            final AffineTransform at = new AffineTransform();
            at.translate(WIDTH / 2.0 - 200.0 - cameraX * zoom,
                    HEIGHT / 2.0 + cameraY * zoom);
            at.scale(zoom, -zoom);
            g.setTransform(at);
            paintFloor(g);
            paintCars(g);
            g.setTransform(new AffineTransform());
            paintInterface(g);
        }
    }

    private void paintCars(final Graphics2D g) {
        for (int i = cars.size() - 1; i >= 0; i--) {
            final Car car = cars.get(i);
            if (car == null || !car.alive)
                continue;

            final Color wStroke = new Color(0x444444);
            g.setStroke(new BasicStroke(1f / zoom));

            for (final Body b : car.wheels)
                for (Fixture f = b.getFixtureList(); f != null; f = f.m_next) {
                    final CircleShape s = (CircleShape) f.getShape();
                    final int wFill = Math.round(255f - 255f
                            * (f.m_density - WHEEL_MINDENS) / WHEEL_MAXDENS);
                    paintCircle(g, b, s.m_p, s.m_radius, b.m_sweep.a, wStroke,
                            new Color(wFill, wFill, wFill));
                }

            final int cDensity = Math.round(
                    100f - 70f * ((car.carDef.chassDensity - CHASS_MINDENS)
                            / CHASS_MAXDENS));
            final Color cFill = Color.getHSBColor(car.carDef.chassColor, 0.5f,
                    cDensity / 100f);
            final Color cStroke = car.elite ? cFill.darker() : cFill.brighter();

            final Body b = car.chassis;
            for (Fixture f = b.getFixtureList(); f != null; f = f.m_next) {
                final PolygonShape s = (PolygonShape) f.getShape();
                paintPolygon(g, b, s.m_vertices, s.m_count, cStroke, cFill);
            }
        }
    }

    private void paintCircle(final Graphics2D g, final Body body,
            final Vec2 center, final float radius, final float angle,
            final Color stroke, final Color fill) {
        final Vec2 p = body.getWorldPoint(center);

        final Ellipse2D.Float ellipse = new Ellipse2D.Float(p.x - radius,
                p.y - radius, 2f * radius, 2f * radius);
        g.setColor(fill);
        g.fill(ellipse);
        g.setColor(stroke);
        g.draw(ellipse);

        final Line2D.Float line = new Line2D.Float(p.x, p.y,
                p.x + (float) (radius * Math.cos(angle)),
                p.y + (float) (radius * Math.sin(angle)));
        g.draw(line);
    }

    private void paintFloor(final Graphics2D g) {
        final List<Vec2> vertices = new ArrayList<Vec2>();
        vertices.addAll(floorVerts);

        final GeneralPath sky = new GeneralPath();
        sky.moveTo(-5f, 0f);

        final GeneralPath ground = new GeneralPath();
        ground.moveTo(-5f, 0f);

        for (final Vec2 point : vertices) {
            sky.lineTo(point.x, point.y);
            ground.lineTo(point.x, point.y);
        }

        final Vec2 first = vertices.get(0);
        final Vec2 last = vertices.get(vertices.size() - 1);

        final float yMax = first.y + 50f;
        sky.lineTo(last.x, yMax);
        sky.lineTo(first.x, yMax);
        sky.closePath();

        final float yMin = first.y - 50f;
        ground.lineTo(last.x, yMin);
        ground.lineTo(first.x, yMin);
        ground.closePath();

        g.setColor(new Color(0xbfe4f8));
        g.fill(sky);

        final AffineTransform tr = new AffineTransform();
        tr.translate(0.0, 0.125);
        ground.transform(tr);
        g.setStroke(new BasicStroke(3.5f / zoom));
        g.setColor(new Color(0x9dd495));
        g.fill(ground);
        g.setColor(new Color(0xa9dac6));
        g.draw(ground);

        tr.translate(0.0, -0.375);
        ground.transform(tr);
        g.setColor(new Color(0xd0a776));
        g.fill(ground);
        g.setColor(new Color(0xa58561));
        g.draw(ground);
    }

    private void paintInterface(final Graphics2D g) {
        g.setColor(Color.WHITE);
        g.setFont(new Font("Rockwell", Font.PLAIN, 20));

        g.drawString("Top Scores:", 10, 25);
        Collections.sort(topScores, new Comparator<Score>() {

            @Override
            public int compare(final Score o1, final Score o2) {
                return o1.score > o2.score ? -1 : 1;
            }
        });
        for (int i = 0; i < Math.min(5, topScores.size()); ++i) {
            final Score s = topScores.get(i);
            final int value = (int) (100f - 70f
                    * (s.carDef.chassDensity - CHASS_MINDENS) / CHASS_MAXDENS);
            g.setColor(
                    Color.getHSBColor(s.carDef.chassColor, 0.5f, value / 100f));
            g.drawString(
                    "#" + i + " [" + s.index + "] "
                            + Math.round(s.score * 100.0) / 100.0 + "m",
                    10, 50 + i * 25);
        }

        g.setColor(Color.WHITE);
        g.drawString("Generation: " + generation, 10, HEIGHT - 60);
        g.drawString("Cars Alive: " + (POP_COUNT - deadCars) + "/" + POP_COUNT,
                10, HEIGHT - 35);

        String s = "Distance: " + distance + "m";
        Dimension d = getStringBounds(g, g.getFont(), s);
        g.drawString(s, WIDTH - 10 - d.width, HEIGHT - 60);
        s = "Max Distance: " + maxDistance + "m";
        d = getStringBounds(g, g.getFont(), s);
        g.drawString(s, WIDTH - 10 - d.width, HEIGHT - 35);
    }

    private void paintPolygon(final Graphics2D g, final Body body,
            final Vec2[] vtx, final int vtxCnt, final Color stroke,
            final Color fill) {
        final GeneralPath path = new GeneralPath();
        final Vec2 p0 = body.getWorldPoint(vtx[0]);
        path.moveTo(p0.x, p0.y);

        for (int i = 1; i < vtxCnt; ++i) {
            final Vec2 p = body.getWorldPoint(vtx[i]);
            path.lineTo(p.x, p.y);
        }
        path.lineTo(p0.x, p0.y);

        g.setStroke(new BasicStroke(1f / zoom));
        g.setColor(fill);
        g.fill(path);
        g.setColor(stroke);
        g.draw(path);
    }

    private Vec2[] rotateFloorTile(final Vec2[] coords, final Vec2 center,
            final float angle) {
        final float c = (float) Math.cos(angle);
        final float s = (float) Math.sin(angle);
        final Vec2[] newCoords = new Vec2[coords.length];
        for (int i = 0; i < coords.length; ++i) {
            final float x = c * (coords[i].x - center.x)
                    - s * (coords[i].y - center.y) + center.x;
            final float y = s * (coords[i].x - center.x)
                    + c * (coords[i].y - center.y) + center.y;
            newCoords[i] = new Vec2(x, y);
        }
        return newCoords;
    }

    @Override
    public void run() {
        while (true) {
            synchronized (cars) {
                simulationStep();
            }

            repaint();
            try {
                Thread.sleep(1000L / BOX2D_FPS);
            } catch (final InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void setCameraPosition() {
        if (cameraTarget == -2)
            return;

        final Vec2 cameraTargetPosition;
        if (cameraTarget >= 0)
            cameraTargetPosition = cars.get(cameraTarget).getPosition();
        else
            cameraTargetPosition = leaderPosition;

        final float diffY = cameraY - cameraTargetPosition.y;
        final float diffX = cameraX - cameraTargetPosition.x;
        cameraY -= cameraSpeed * diffY;
        cameraX -= cameraSpeed * diffX;
    }

    private void simulationStep() {
        world.step(1f / BOX2D_FPS, 20, 20);

        for (int i = 0; i < POP_COUNT; ++i) {
            final Car car = cars.get(i);
            if (!car.alive)
                continue;

            car.frames++;

            final Vec2 position = car.getPosition();
            if (car.checkDeath()) {
                car.kill();
                deadCars++;
                if (deadCars >= POP_COUNT)
                    newRound();
                if (leader == i)
                    findLeader();
                continue;
            }

            if (position.x > leaderPosition.x) {
                leaderPosition = position;
                leader = i;
            }
        }

        distance = Math.round(leaderPosition.x * 100f) / 100f;
        maxDistance = Math.max(maxDistance, distance);
    }
}
