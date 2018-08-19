enum Orientation {
    FOWARDS, BACKWARDS;
    
    private static boolean isFowards(double heading) {
        if((heading >= 0 && heading <= Math.PI / 2) || (heading > (3 * Math.PI) / 2 && heading <= 2 * Math.PI)) {
            return true; 
        }else {
            return false; 
        }
    }

    private static boolean isBackwards(double heading) {
        if(heading > Math.PI && heading <= (3 * Math.PI) / 2) {
            return true; 
        }else {
            return false; 
        }
    }

    public static Orientation findOrientation(double heading) {
        if(isFowards(heading)) {
            return Orientation.FOWARDS; 
        }else {
            return Orientation.BACKWARDS; 
        }
    }
}