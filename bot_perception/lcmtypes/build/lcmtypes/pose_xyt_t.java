/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class pose_xyt_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public float x;
    public float y;
    public float theta;
 
    public pose_xyt_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x0f98bd7892313b56L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.pose_xyt_t.class))
            return 0L;
 
        classes.add(lcmtypes.pose_xyt_t.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.utime); 
 
        outs.writeFloat(this.x); 
 
        outs.writeFloat(this.y); 
 
        outs.writeFloat(this.theta); 
 
    }
 
    public pose_xyt_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public pose_xyt_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.pose_xyt_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.pose_xyt_t o = new lcmtypes.pose_xyt_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.x = ins.readFloat();
 
        this.y = ins.readFloat();
 
        this.theta = ins.readFloat();
 
    }
 
    public lcmtypes.pose_xyt_t copy()
    {
        lcmtypes.pose_xyt_t outobj = new lcmtypes.pose_xyt_t();
        outobj.utime = this.utime;
 
        outobj.x = this.x;
 
        outobj.y = this.y;
 
        outobj.theta = this.theta;
 
        return outobj;
    }
 
}
