/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors;


import java.awt.*;
import java.util.Optional;
import java.awt.image.BufferedImage;
import java.awt.Color;
import java.awt.Graphics2D;


public class Drawer {

    private static int width = 320;
    private static int height = 280;

    private Optional<Image> value = Optional.empty();
    private BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
    private Graphics2D g2 = (Graphics2D) image.getGraphics();


    public void drawBackground() {
        // draw sky
        g2.setColor(Color.CYAN);
        g2.drawRect(0, 0, 320, 80);
        g2.fillRect(0, 0, 320, 80);

        // draw ground
        g2.setColor(Color.LIGHT_GRAY);
        g2.drawRect(0, 81, 320, 200);
        g2.fillRect(0, 81, 320, 200);

        // draw street
        int[] x = {127, 191, 255, 63};
        int[] y = {81, 81, 319, 319};
        g2.setColor(Color.WHITE);
        g2.drawPolygon(x, y, 4);
        g2.fillPolygon(x, y, 4);
    }


    public void drawIntersection(double distFrontLeftToInter, double distFrontRightToInter, double distBackLeftToInter, double distBackRightToInter, double distToInter) {
        int interWidth = 320;
        int interLength = 32;

        boolean inFront = false;
        if (distFrontLeftToInter < distBackLeftToInter && (distFrontLeftToInter - distFrontRightToInter) < 16 && (distFrontRightToInter - distFrontLeftToInter) < 16)
            inFront = true;
        int interUpperEdge = (int) (-199.0 / 100.0 * distToInter + 279);


        if (inFront) {
            g2.setColor(Color.WHITE);
            g2.drawRect(0, interUpperEdge, interWidth, interLength);
            g2.fillRect(0, interUpperEdge, interWidth, interLength);
        }
    }

    public void drawVehicle(double dist, double distFrontLeftToCar, double distBackLeftToCar) {

        // detect whether the car is in front of us
        boolean inFront = true;
        if (distBackLeftToCar < distFrontLeftToCar) inFront = false;

        int carWidth = 16;
        int carLength = 30;

        // the left edge of the car
        int car_leftEdge = 151;
        // the head edge of the car
        int car_upperEdge = (int) (-199.0 / 110.0 * dist + 279.0);

        // the detected car is infront of us
        if (inFront) {
            g2.setColor(Color.PINK);
            g2.drawRect(car_leftEdge, car_upperEdge, carWidth, carLength);
            g2.fillRect(car_leftEdge, car_upperEdge, carWidth, carLength);
        }
    }

    public void drawBuilding(double dist, double distFrontLeftToBuilding, double distFrontRightToBuilding, double distBackLeftToBuilding, double distBackRightToBuilding) {

        // detect whether the vehicle has already passed the building
        boolean onTheLeft = true;
        if (distFrontLeftToBuilding < distFrontRightToBuilding) onTheLeft = false;
        // detect whether the building is on the left side or the right side
        boolean inFront = true;
        if (distBackLeftToBuilding < distFrontLeftToBuilding) inFront = false;

        // the left edge of the building on the left hand side of the street
        int rightBuilding_leftEdge = (int) (-64.0 / 100 * dist + 287);
        // the right edge of the building on the right hand side of the street
        int leftBuliding_RightEdge = (int) (64.0 / 110 * dist + 31);
        int buildingUpperEdge = (int) (-239.0 / 110 * dist + 319);

        int buidlingWidth = 64;
        int buildingHeight = 32;

        // draw building
        // building on the left hand side
        if (onTheLeft && inFront) {
            g2.setColor(Color.BLACK);
            g2.drawRect(leftBuliding_RightEdge - buidlingWidth, buildingUpperEdge, buidlingWidth, buildingHeight);
            g2.fillRect(leftBuliding_RightEdge - buidlingWidth, buildingUpperEdge, buidlingWidth, buildingHeight);
        }
        // building on the right hand side
        else if (!onTheLeft && inFront) {
            g2.setColor(Color.BLACK);
            g2.drawRect(rightBuilding_leftEdge, buildingUpperEdge, buidlingWidth, buildingHeight);
            g2.fillRect(rightBuilding_leftEdge, buildingUpperEdge, buidlingWidth, buildingHeight);
        }

    }

    public void drawTree(double dist, double distFrontLeftToTree, double distFrontRightToTree, double distBackLeftToTree, double distBackRightToTree) {
        // detect whether the tree is on the left side or the right side
        boolean onTheLeft = true;
        if (distFrontLeftToTree < distFrontRightToTree) onTheLeft = false;
        // detect whether the vehicle has already passed the tree
        boolean inFront = true;
        if (distBackLeftToTree < distFrontLeftToTree) inFront = false;

        // the left edge of the Tree on the left hand side of the street
        int rightTree_leftEdge = (int) (-64.0 / 100 * dist + 287);
        // the right edge of the Tree on the right hand side of the street
        int leftTree_RightEdge = (int) (64.0 / 110 * dist + 31);
        int treeUpperEdge = (int) (-239.0 / 110 * dist + 319);

        int treeWidth = 6;
        int treeHeight = 25;

        // draw Tree
        // Tree on the left hand side
        if (onTheLeft && inFront) {
            g2.setColor(Color.YELLOW);
            g2.drawRect(leftTree_RightEdge - treeWidth + 15, treeUpperEdge + 16, treeWidth, treeHeight);
            g2.fillRect(leftTree_RightEdge - treeWidth + 15, treeUpperEdge + 16, treeWidth, treeHeight);
            g2.setColor(Color.GREEN);
            g2.drawOval(leftTree_RightEdge - treeWidth + 15 - treeWidth * 2, treeUpperEdge + 16 - treeWidth * 2, 5 * treeWidth, 4 * treeWidth);
            g2.fillOval(leftTree_RightEdge - treeWidth + 15 - treeWidth * 2, treeUpperEdge + 16 - treeWidth * 2, 5 * treeWidth, 4 * treeWidth);
        }
        // tree on the right hand side
        else if (!onTheLeft && inFront) {
            g2.setColor(Color.YELLOW);
            g2.drawRect(rightTree_leftEdge - 15, treeUpperEdge + 16, treeWidth, treeHeight);
            g2.fillRect(rightTree_leftEdge - 15, treeUpperEdge + 16, treeWidth, treeHeight);
            g2.setColor(Color.GREEN);
            g2.drawOval(rightTree_leftEdge - 8 - treeWidth * 3, treeUpperEdge + 16 - treeWidth * 2, 5 * treeWidth, 4 * treeWidth);
            g2.fillOval(rightTree_leftEdge - 8 - treeWidth * 3, treeUpperEdge + 16 - treeWidth * 2, 5 * treeWidth, 4 * treeWidth);
        }
    }


    public Optional<Image> getImage() {
        Image i = image;
        Optional<Image> im = Optional.of(i);
        value = im;
        return value;
    }

    public BufferedImage getBImage() {
        return image;
    }

    public static void main() {

        Drawer drawer = new Drawer();
        drawer.drawBackground();
        BufferedImage im = drawer.getBImage();
        System.out.print(im == null);

    }
}
