package com.team254.frc2024.subsystems.vision;

import java.nio.ByteBuffer;

import com.team254.lib.limelight.LimelightHelpers;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class FiducialObservation implements StructSerializable {
    public static class FiducialObservationStruct implements Struct<FiducialObservation> {
        public int id;
        public double txnc;
        public double tync;
        public double ambiguity;

        @Override
        public Class<FiducialObservation> getTypeClass() {
            return FiducialObservation.class;
        }

        @Override
        public String getTypeString() {
            return "struct:FiducialObservation";
        }

        @Override
        public int getSize() {
            return kSizeInt32 + 3 * kSizeDouble;
        }

        @Override
        public String getSchema() {
            return "int id;double txnc;double tync;double ambiguity";
        }

        @Override
        public FiducialObservation unpack(ByteBuffer bb) {
            FiducialObservation rv = new FiducialObservation();
            rv.id = bb.getInt();
            rv.txnc = bb.getDouble();
            rv.tync = bb.getDouble();
            rv.ambiguity = bb.getDouble();
            return rv;
        }

        @Override
        public void pack(ByteBuffer bb, FiducialObservation value) {
            bb.putInt(value.id);
            bb.putDouble(value.txnc);
            bb.putDouble(value.tync);
            bb.putDouble(value.ambiguity);
        }
    }

    public int id;
    public double txnc;
    public double tync;
    public double ambiguity;

    public FiducialObservation() {
    }

    public static FiducialObservation fromLimelight(LimelightHelpers.RawFiducial fiducial) {
        FiducialObservation rv = new FiducialObservation();
        rv.id = fiducial.id;
        rv.txnc = fiducial.txnc;
        rv.tync = fiducial.tync;
        rv.ambiguity = fiducial.ambiguity;

        return rv;
    }

    public static FiducialObservation[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
        FiducialObservation[] rv = new FiducialObservation[fiducials.length];
        for (int i = 0; i < fiducials.length; ++i) {
            rv[i] = fromLimelight(fiducials[i]);
        }
        return rv;
    }

    public static final FiducialObservationStruct struct = new FiducialObservationStruct();
}
