package com.team980.robot2019.vision;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

import java.util.List;

/**
 * Implements VisionDataProvider to calculate and provide data from the back Pixy.
 */
@Deprecated //We're probably not going to use the rear camera this season.
public class BackCameraProcessor implements VisionDataProvider {

    private static final int VISION_TARGET_SIGNATURE = 1;

    private Pixy2 pixy;
    private List<Pixy2CCC.Block> blocks;

    private double targetCenterCoord = -1;
    private double targetWidth = -1;

    public BackCameraProcessor() {
        pixy = Pixy2.createInstance(Pixy2.LinkType.SPI);
        pixy.init();

        blocks = pixy.getCCC().getBlocks();
    }

    /**
     * Receives blocks from Pixy and updates the resulting data.
     */
    public void updateData() {
        int numBlocks = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 3); //10 causes an intermittent memory leak...

        if (numBlocks == 1) {
            Pixy2CCC.Block target = blocks.get(0);

            targetCenterCoord = target.getX();
            targetWidth = target.getWidth();
        } else if (numBlocks >= 2) {
            // WARNING: This does not check to make sure they are the correct signature!
            Pixy2CCC.Block largestTarget = blocks.get(0);
            Pixy2CCC.Block secondLargestTarget = blocks.get(1);

            if (area(secondLargestTarget) > area(largestTarget)) {
                // Swap the two blocks
                Pixy2CCC.Block _tmp = secondLargestTarget;
                secondLargestTarget = largestTarget;
                largestTarget = _tmp;
            }

            for (int i = 2; i < numBlocks; i++) {
                Pixy2CCC.Block block = blocks.get(i);

                if (block.getSignature() == VISION_TARGET_SIGNATURE) {
                    if (area(block) > area(largestTarget)) {
                        secondLargestTarget = largestTarget;
                        largestTarget = block;
                    } else if (area(block) > area(secondLargestTarget)) {
                        secondLargestTarget = block;
                    }
                }
            }

            targetCenterCoord = (largestTarget.getX() + secondLargestTarget.getX()) / 2.0d;

            // Add the two target widths, plus the distance between them - overapproximation
            targetWidth = largestTarget.getWidth() + secondLargestTarget.getWidth() + Math.abs(largestTarget.getX() - secondLargestTarget.getX());
        }
    }

    private int area(Pixy2CCC.Block block) {
        return block.getWidth() * block.getHeight();
    }

    @Override
    public String getSource() {
        return "roboRIO-API";
    }

    @Override
    public double getTargetCenterCoord() {
        return targetCenterCoord;
    }

    @Override
    public double getTargetCenterOffset() {
        return targetCenterCoord - 160 - 25; //Configure if we resurrect back pixy
    }

    @Override
    public double getTargetWidth() {
        return targetWidth;
    }
}