package com.platypus.android.server;

import android.util.Log;

import com.platypus.crw.VehicleServer;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ThreadLocalRandom;

public class PointOfInterest
{
    double[] location;
    String desc;
    VehicleServer.MapMarkerTypes type;
    long id;

    private static String logTag = "POI";
    private static final Object poi_lock = new Object();
    private static Map<Long, PointOfInterest> poi_by_index = new HashMap<>();
    private static Map<Long, PointOfInterest> unsent_poi = new HashMap<>();

    PointOfInterest(long _id, double[] _location, VehicleServer.MapMarkerTypes _type, String _desc)
    {
        id = _id;
        location = _location.clone();
        type = _type;
        desc = _desc;
    }

    static long newPOI(double[] _location, VehicleServer.MapMarkerTypes _type, String _desc)
    {
        synchronized (poi_lock)
        {
            long new_index = poi_by_index.size();
            PointOfInterest new_poi = new PointOfInterest(new_index, _location, _type, _desc);
            Log.v(logTag, String.format("Generating new POI #%d", new_index));
            poi_by_index.put(new_index, new_poi);
            unsent_poi.put(new_index, new_poi);
            return new_index;
        }
    }

    static void acknowledge(long _id)
    {
        Log.d(logTag, String.format("POI #%d was acknowledged", _id));
        synchronized (poi_lock)
        {
            if (unsent_poi.containsKey(_id))
            {
                Log.v(logTag, "removing acknowledged POI from unsent list");
                unsent_poi.remove(_id);
            }
        }
    }

    static PointOfInterest getRandomPOI()
    {
        synchronized (poi_lock)
        {
            if (unsent_poi.size() < 1) return null;
            Long[] unsent_ids = unsent_poi.keySet().toArray(new Long[0]);
            int random_index = ThreadLocalRandom.current().nextInt(0, unsent_poi.size());
            long index = unsent_ids[random_index];
            return unsent_poi.get(index);
        }
    }

    static PointOfInterest getPOIByIndex(long index)
    {
        synchronized (poi_lock)
        {
            return poi_by_index.get(index);
        }
    }
}
