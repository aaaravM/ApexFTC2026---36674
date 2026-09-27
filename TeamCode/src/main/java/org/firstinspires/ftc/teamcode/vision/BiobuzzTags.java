package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * The BIOBUZZ AprilTag cluster geometry, as constants.
 *
 * <h2>Where these numbers came from</h2>
 * They were extracted from the <b>compiled Vision-12.0.0 SDK binary</b>, not from a web page. That
 * was necessary because the geometry is not reachable through the public API: {@code
 * AprilTagClusterMetadata} declares <b>zero methods</b> and only five public fields (name, shortName,
 * distanceUnit, fieldPosition, fieldOrientation). Its {@code clusterMembers} list appears solely as a
 * constructor parameter, so there is no public getter and nothing in the Javadoc.
 *
 * <p>Having them here means you do not need a field, a dump OpMode, or a Limelight to start writing
 * aiming code.
 *
 * <h2>The four clusters</h2>
 * <table>
 *   <tr><th>Cluster</th><th>Tag IDs</th></tr>
 *   <tr><td>RED SCORING</td><td>30, 31, 32, 33</td></tr>
 *   <tr><td>RED AUDIENCE</td><td>34, 35, 36, 37</td></tr>
 *   <tr><td>BLUE AUDIENCE</td><td>38, 39, 40, 41</td></tr>
 *   <tr><td>BLUE SCORING</td><td>42, 43, 44, 45</td></tr>
 * </table>
 * Every tag is <b>3.25 in</b> (82.55 mm). Every cluster's {@code fieldPosition} is {@code (0,0,0)}
 * with identity orientation -- which is machine-readable confirmation of the release note that these
 * tags are not for field localisation. They move, so there is no field position to record.
 *
 * <h2>The member offsets, and the useful symmetry</h2>
 * All four members of a cluster sit at {@code (x, 7.1874, -5.622)} inches relative to the cluster
 * origin, with {@code x} in <b>{-6.5, -2.75, +2.75, +6.5}</b>.
 *
 * <p>Two things follow, and the first is genuinely handy:
 * <ul>
 *   <li><b>The x offsets are symmetric and sum to zero.</b> So if you can see all four tags, the
 *       average of their positions already <i>is</i> the cluster origin in x -- no per-tag correction
 *       needed at all. If you see a subset, correct by the mean x of just the tags you can see.</li>
 *   <li><b>The y and z offsets are identical for every member.</b> So that part of the correction is
 *       a constant, whatever subset you happen to see.</li>
 * </ul>
 */
public final class BiobuzzTags {

    private BiobuzzTags() {}

    /** Tag size, inches. The Limelight needs this in MILLIMETRES -- see {@link #TAG_SIZE_MM}. */
    public static final double TAG_SIZE_IN = 3.25;

    /**
     * Tag size in millimetres, for a Limelight {@code LLFieldMap} fiducial entry.
     *
     * <p>The default in {@code LLFieldMap.Fiducial} is <b>165.1 mm</b> -- the FRC 6.5-inch tag.
     * Leaving that default in place gives a pose scaled by roughly 2x, so the turret would range the
     * HIVE at twice its real distance.
     */
    public static final double TAG_SIZE_MM = TAG_SIZE_IN * 25.4;   // 82.55

    /** Member x offsets from the cluster origin, inches, in tag-ID order within a cluster. */
    public static final double[] MEMBER_X_OFFSET_IN = {-6.5, -2.75, 2.75, 6.5};

    /** Every member's y offset from the cluster origin, inches. Constant across members. */
    public static final double MEMBER_Y_OFFSET_IN = 7.1874;

    /** Every member's z offset from the cluster origin, inches. Constant across members. */
    public static final double MEMBER_Z_OFFSET_IN = -5.622;

    /** The four BIOBUZZ CELL clusters, with the tag IDs each contains. */
    public enum Cluster {
        RED_SCORING("RED SCORING", 30, RobotConstants.Alliance.RED),
        RED_AUDIENCE("RED AUDIENCE", 34, RobotConstants.Alliance.RED),
        BLUE_AUDIENCE("BLUE AUDIENCE", 38, RobotConstants.Alliance.BLUE),
        BLUE_SCORING("BLUE SCORING", 42, RobotConstants.Alliance.BLUE);

        /** The cluster's {@code shortName} as the SDK's tag library reports it. */
        public final String shortName;
        /** Lowest tag ID in this cluster. The four members are base..base+3. */
        public final int baseTagId;
        /** Which alliance this CELL belongs to. */
        public final RobotConstants.Alliance alliance;

        Cluster(String shortName, int baseTagId, RobotConstants.Alliance alliance) {
            this.shortName = shortName;
            this.baseTagId = baseTagId;
            this.alliance = alliance;
        }

        public boolean contains(int tagId) {
            return tagId >= baseTagId && tagId <= baseTagId + 3;
        }

        /** 0-3, which member of this cluster a tag ID is. -1 if it is not a member. */
        public int memberIndex(int tagId) {
            return contains(tagId) ? tagId - baseTagId : -1;
        }
    }

    /** The cluster a tag ID belongs to, or null if the ID is not a BIOBUZZ CELL tag. */
    public static Cluster clusterOf(int tagId) {
        for (Cluster c : Cluster.values()) {
            if (c.contains(tagId)) return c;
        }
        return null;
    }

    /**
     * The x offset of a tag from its cluster origin, inches, or {@link Double#NaN} if the ID is not a
     * CELL tag.
     *
     * <p>Assumes member order follows tag-ID order within a cluster, which is the natural reading of
     * the extracted data. If aiming is consistently biased sideways by a few inches with only one
     * tag visible, but correct with all four visible, this assumption is what to re-check -- the
     * all-four case averages the error away.
     */
    public static double memberXOffsetIn(int tagId) {
        Cluster c = clusterOf(tagId);
        if (c == null) return Double.NaN;
        return MEMBER_X_OFFSET_IN[c.memberIndex(tagId)];
    }

    /** True if this tag belongs to a CELL our alliance should be shooting into. */
    public static boolean isOurCell(int tagId) {
        Cluster c = clusterOf(tagId);
        return c != null && c.alliance == RobotConstants.alliance;
    }
}
