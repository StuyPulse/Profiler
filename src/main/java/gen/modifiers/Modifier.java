package gen.modifiers;

import gen.Trajectory;

/**
 * Modifier.java
 *
 * @author Tahsin Ahmed
 *
 * Modifies the center trajectory so it could be used in applications.
 */

public abstract class Modifier {

    public final Trajectory original;

    /**
     * @param original the basis trajectory
     */
    public Modifier(Trajectory original) {
        this.original = original;
    }

}
