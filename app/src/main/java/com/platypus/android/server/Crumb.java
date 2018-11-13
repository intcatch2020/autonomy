package com.platypus.android.server;

import android.util.Log;

import com.platypus.crw.data.UtmPose;

import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;

import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TimerTask;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Created by jason on 6/7/17.
 */

// TODO: LIST OF IMPROVEMENTS (NOT IN ANY PARTICULAR ORDER)
// 1) Mechanism to remove pre-existing, redundant waypoints if we have too many
// 2) checkForNewCrumb does not modify the graph each time, but instead just pushes the current location into a buffer, with the only requirement being at least X meters from the most recent location
//      There is a consumer thread that will periodically pull location(s) from the buffer, check if it is worthy of being a crumb, and adds it to the graph.
//      If there is nothing for the consumer to do, it should sleep for 1 second or something similar.
//      We should assume that the consumer thread will be slower than the production of new locations, especially when the graph grows in size.
//      Thus we need this buffer method to guarantee the

//      When A* is called, it needs to modify the graph with anything currently in the buffer, then add start and goal as new crumbs too
//      The reasoning behind all this: the boat moves between checking for new crumbs.
//      If it takes a long time to modify the graph, the boat may move too far by the time a new crumb can be created.
//      By "too far", I mean that the new crumb will NOT be connected to the previous one, the graph is not complete, and A* may fail
//      This buffer method should still give us the benefit of building the graph as we go, but minimizing the synchronization/blocking
//      thus minimizing the chance that the graph is incomplete.
//      To do this, we can use a consumer thread that pops a location from the buffer and modifies the graph (or waits for X locations first)
//      We already have a producer thread that will generate new locations.


public class Crumb
{
		// Instance fields
		private long index;
		private UTM location;
		private double g = 0.; // initialize with zero
		private double h; // run-time initialize e.g. set to the distance to the goal (if using A*)
		private List<Long> successors = new ArrayList<>();
		private long parent;

		// Instance methods
		private Crumb(long _index, UTM _location)
		{
				index = _index;
				location = _location;
		}
		private void setG(double _g) { g = _g; }
		private double getG() { return g; }
		private void setH(double _h) { h = _h; }
		private double getH() { return h;}
		private void setParent(long _parent) { parent = _parent; }
		private long getParent() { return parent; }
		private double getCost() { return g + h; }
		long getIndex() { return index; }
		UTM getLocation() { return location; }
		public List<Long> getSuccessors() { return successors; }


		// Static fields and methods
		private static ArrayList<UtmPose> crumb_buffer = new ArrayList<>();
		private static final Object crumbs_buffer_lock = new Object();
		private final static int max_buffer_size = 100; // don't push into the buffer if you reach this size, or pop the oldest first
		private final static double MAX_NEIGHBOR_DISTANCE = 5;
		private final static double REDUNDANT_DISTANCE = 0.5; // if less than this apart from any current crumb, do not generate new crumb
		private static Map<Long, Crumb> crumbs_by_index = new HashMap<>();
		private static Map<Long, Crumb> unsent_crumbs = new HashMap<>();
		private static Map<Long, Map<Long, Double>> pairwise_distances = new HashMap<>();
		private static Map<Long, List<Long>> neighbors = new HashMap<>();
		private static final Object crumbs_lock = new Object();
		private static String logTag = "crumbs";

		static class CrumbBufferConsumptionRunnable implements Runnable
		{
			UtmPose potential_crumb = null;

			@Override
			public void run()
			{
				potential_crumb = null;
				synchronized (crumbs_buffer_lock)
				{
					if (crumb_buffer.size() > 0)
					{
						//Log.v(logTag, "There is at least one potential crumb location to evaluate");
						potential_crumb = crumb_buffer.get(0).clone();
						Log.d(logTag, String.format("Checking location %s", potential_crumb.toString()));
						crumb_buffer.remove(0);
					}
				}

				// if the buffer was not full enough, do nothing but sleep for one second
				if (potential_crumb == null)
				{
					try
					{
						Log.v(logTag, "There were no potential crumbs. Sleeping...");
						Thread.sleep(1000);
					}
					catch (Exception e)
					{
						Log.e(logTag, String.format("Crumb buffer consumption error: %s", e.getMessage()));
					}
				}
				else
				{
					// now, we can do much slower things without worry, because the crumb buffer can safely fill up while we work
					// check if the potential crumb is 1) far enough away from the last crumb, and 2) not very close to any crumb
					synchronized (crumbs_lock) {
						try {
							if (potential_crumb.isDefault()) return; // ignore default location
							UTM new_utm = UTM.valueOf(
									potential_crumb.origin.zone,
									potential_crumb.origin.isNorth ? 'T' : 'L',
									potential_crumb.pose.getX(),
									potential_crumb.pose.getY(),
									SI.METER
							);

							long last_index = crumbs_by_index.size();
							if (last_index < 1) {
								Log.i(logTag, "Generating first crumb");
								newCrumb(new_utm, distanceFromAllCurrentCrumbs(new_utm));
								return;
							}
							Log.v(logTag, String.format("Checking to create a new crumb..."));
							Crumb last_crumb = crumbs_by_index.get(last_index - 1);
							UTM last_utm = last_crumb.getLocation();
							double distance = Math.pow(potential_crumb.pose.getX() - last_utm.eastingValue(SI.METER), 2.0);
							distance += Math.pow(potential_crumb.pose.getY() - last_utm.northingValue(SI.METER), 2.0);
							distance = Math.sqrt(distance);
							if (distance >= 0.75 * MAX_NEIGHBOR_DISTANCE)
							{
								// note the 0.75 coefficient. This ensures the crumbs are generated a little closer than the cutoff for neighbors

								// execute a slower check that makes sure you are not very close to any crumb
								Map<Long, Double> new_pairwise_distances = distanceFromAllCurrentCrumbs(new_utm);
								for (double d : new_pairwise_distances.values())
								{
									if (d <= REDUNDANT_DISTANCE)
									{
										Log.d(logTag, String.format("No new crumb, location was redundant (dist = %.2f)", d));
										return;
									}
								}
								Log.d(logTag, String.format("Generating new crumb #%d", last_index));
								newCrumb(new_utm, new_pairwise_distances);
							}
							else
							{
								Log.v(logTag, String.format("Distance from last crumb = %.2f, insufficient", Math.sqrt(distance)));
							}
						}
						catch (Exception e)
						{
							Log.e(logTag, String.format("Crumb buffer consumption error: %s", e.getMessage()));
						}
					}
				}
			}
		}

		static void checkForNewCrumb(UtmPose current_utmpose)
		{
			synchronized (crumbs_buffer_lock)
			{
				if (!crumb_buffer.isEmpty())
				{
					// check that you aren't resending the exact same thing repeatedly
					if (distanceBetweenUtmPose(current_utmpose, crumb_buffer.get(crumb_buffer.size() - 1)) < REDUNDANT_DISTANCE) {
						Log.v(logTag, "Location is already in the crumbs buffer, ignoring...");
						return;
					}
				}

				// TODO: if the buffer is getting too large, need to start throwing away old points. Or would it be better to throw away something else?
				// TODO: if (crumb_buffer.size() >= max_buffer_size) crumb_buffer.remove(0);
				Log.d(logTag, String.format("pushing %s into crumb buffer", current_utmpose.toString()));
				crumb_buffer.add(current_utmpose);
			}
		}
		static Crumb getRandomCrumb()
		{
				synchronized (crumbs_lock)
				{
						if (unsent_crumbs.size() < 1) return null;
						Long[] unsent_ids = unsent_crumbs.keySet().toArray(new Long[0]);
						int random_index = ThreadLocalRandom.current().nextInt(0, unsent_crumbs.size());
						long index = unsent_ids[random_index];
						Log.d(logTag, String.format("Random crumb: #%d", index));
						return unsent_crumbs.get(index);
				}
		}

		static void acknowledge(long _id)
		{
				synchronized (crumbs_lock)
				{
					// TODO: apparently unsent_crumbs.remove(_id) isn't working, because the phone keeps resending
					Log.d(logTag, String.format("Crumb #%d was acknowledged", _id));
					if (unsent_crumbs.containsKey(_id))
					{
						Log.v(logTag, "removing acknowledged crumb from unsent list");
						unsent_crumbs.remove(_id);
					}
				}
		}

		private static double distanceBetweenUtmPose(UtmPose location_i, UtmPose location_j)
		{
			double dx = location_i.pose.getX() - location_j.pose.getX();
			double dy = location_i.pose.getY() - location_j.pose.getY();
			return Math.sqrt(dx*dx + dy*dy);
		}


		private static double distanceBetweenUTM(UTM location_i, UTM location_j)
		{
			double dx = location_i.eastingValue(SI.METER) - location_j.eastingValue(SI.METER);
			double dy = location_i.northingValue(SI.METER) - location_j.northingValue(SI.METER);
			return Math.sqrt(dx*dx + dy*dy);
		}

		private static double distanceBetweenCrumbs(long index_i, long index_j)
		{
				UTM location_i = crumbs_by_index.get(index_i).getLocation();
				UTM location_j = crumbs_by_index.get(index_j).getLocation();
				return distanceBetweenUTM(location_i, location_j);
		}

		private static Map<Long, Double> distanceFromAllCurrentCrumbs(UTM _location)
		{
			Map<Long, Double> result = new HashMap<Long, Double>();
			synchronized (crumbs_lock)
			{
				for (Map.Entry<Long, Crumb> old_entry : crumbs_by_index.entrySet())
				{
					long old_index = old_entry.getKey();
					result.put(old_index, distanceBetweenUTM(_location, old_entry.getValue().getLocation()));
				}
				return result;
			}
		}

		private static long newCrumb(UTM _location, Map<Long, Double> new_pairwise_distances)
		{
			// modify the graph of crumbs
			synchronized (crumbs_lock)
			{
					// initialize objects
					long new_index = crumbs_by_index.size();
					Crumb new_crumb = new Crumb(new_index, _location);
					crumbs_by_index.put(new_index, new_crumb);
					unsent_crumbs.put(new_index, new_crumb);
					pairwise_distances.put(new_index, new HashMap<Long, Double>());
					neighbors.put(new_index, new ArrayList<Long>());

					for (Map.Entry<Long, Crumb> old_entry : crumbs_by_index.entrySet())
					{
						long old_index = old_entry.getKey();
						if (old_index == new_index) continue; // need to make sure that new index isn't checked against itself!
						double distance = new_pairwise_distances.get(old_index);
						if (distance <= MAX_NEIGHBOR_DISTANCE)
						{
							Log.v(logTag, String.format("New crumb #%d is neighbors with crumb #%d", new_index, old_index));
							pairwise_distances.get(old_index).put(new_index, distance);
							pairwise_distances.get(new_index).put(old_index, distance);
							neighbors.get(old_index).add(new_index);
							neighbors.get(new_index).add(old_index);
						}
					}

					if (neighbors.get(new_index).isEmpty())
					{
						Log.w(logTag, String.format("WARNING: New crumb #%d has no neighbors!", new_index));
					}
					else if (neighbors.get(new_index).size() == 1)
					{
						Log.w(logTag, String.format("WARNING: New crumb #%d has only 1 neighbor", new_index));
					}

					return new_index;
			}
		}

		public static List<Long> straightHome(UTM start, UTM goal)
		{
			synchronized (crumbs_lock)
			{
				// Simple: go straight home from the start
				List<Long> path_sequence = new ArrayList<>();
				long start_index = newCrumb(start, distanceFromAllCurrentCrumbs(start));
				long goal_index = newCrumb(goal, distanceFromAllCurrentCrumbs(goal));
				path_sequence.add(start_index);
				path_sequence.add(goal_index);
				return path_sequence;
			}
		}

		public static List<Long> aStar(UTM start, UTM goal) throws Exception
		{
			synchronized (crumbs_lock)
			{
				Log.i("aStar", "Starting A* calculation...");
				long start_index = newCrumb(start, distanceFromAllCurrentCrumbs(start));
				long goal_index = newCrumb(goal, distanceFromAllCurrentCrumbs(goal));
				long original_goal_index = goal_index;
				double original_distance = distanceBetweenCrumbs(start_index, original_goal_index);
				Set<Long> attempted_goals = new HashSet<>();
				Set<Long> open_crumbs = new HashSet<>();
				Set<Long> closed_crumbs = new HashSet<>();
				List<Long> path_sequence = new ArrayList<>();
				boolean goal_is_reachable;
				do {
					attempted_goals.add(goal_index);

					for (Map.Entry<Long, Crumb> entry : crumbs_by_index.entrySet()) {
						double dist_to_goal = distanceBetweenCrumbs(entry.getKey(), goal_index);
						entry.getValue().setH(dist_to_goal);
					}
					open_crumbs.clear();
					closed_crumbs.clear();
					path_sequence.clear();
					long current_crumb = 0;
					long iterations = 0;

					// A*
					open_crumbs.add(start_index);
					Log.d("aStar", String.format("open_crumbs.size() = %d", open_crumbs.size()));

					while (open_crumbs.size() > 0) {
						iterations += 1;
						Log.d("aStar", String.format("A* iter %d:  %d open, %d closed",
								iterations, open_crumbs.size(), closed_crumbs.size()));

						// find open crumb with lowest cost
						double lowest_cost = 99999999;
						for (Long entry : open_crumbs) {
							double cost = crumbs_by_index.get(entry).getCost();
							if (cost < lowest_cost) {
								//Log.v("aStar", String.format("crumb # %d has lowest cost = %.2f", entry.getKey(), cost));
								lowest_cost = cost;
								current_crumb = entry;
							}
						}
						Log.d("aStar", String.format("Current crumb index = %d", current_crumb));
						if (current_crumb == goal_index) {
							Log.i("aStar", String.format("Reached goal crumb %d, exiting loop", goal_index));
							break; // reached goal node, exit loop
						}
						Log.d("aStar", String.format("Current crumb has %d neighbors: %s",
								neighbors.get(current_crumb).size(),
								neighbors.get(current_crumb).toString()));
						for (long s : neighbors.get(current_crumb)) {
							double potential_g = crumbs_by_index.get(current_crumb).getG() + pairwise_distances.get(current_crumb).get(s);
							if (open_crumbs.contains(s)) {
								if (crumbs_by_index.get(s).getG() <= potential_g) continue;
							} else if (closed_crumbs.contains(s)) {
								if (crumbs_by_index.get(s).getG() > potential_g) {
									open_crumbs.add(s);
									closed_crumbs.remove(s);
								} else {
									continue;
								}
							} else {
								open_crumbs.add(s);
							}
							crumbs_by_index.get(s).setG(potential_g);
							crumbs_by_index.get(s).setParent(current_crumb);
						}
						open_crumbs.remove(current_crumb);
						closed_crumbs.add(current_crumb);
					}
					path_sequence.add(current_crumb);
					while (path_sequence.get(0) != start_index) {
						path_sequence.add(0, crumbs_by_index.get(path_sequence.get(0)).getParent());
					}

					// check if final index matches goal index
					goal_is_reachable = path_sequence.get(path_sequence.size()-1) == goal_index;

					// if it does not match, need to select new goal
					if (!goal_is_reachable) {
						// find point closest to goal that
						//   1) has not be attempted
						//   2) is closer to the goal than the start (otherwise what's the point?)
						long new_goal_index = -1;
						double distance_to_goal = 999999;
						for (Map.Entry<Long, Crumb> entry : crumbs_by_index.entrySet()) {
							if (attempted_goals.contains(entry.getKey())) continue;
							double h = entry.getValue().getH();
							if (h < distance_to_goal && h < original_distance) {
								distance_to_goal = h;
								new_goal_index = entry.getKey();
							}
						}
						// handle the scenario where new_goal_index is never set
						if (new_goal_index == -1) throw new Exception("A* ERROR: there are no reachable goal candidates");
						goal_index = new_goal_index;
					}
				} while (!goal_is_reachable);

				// TODO: need to delete goal breadcrumb, otherwise we will violate the purpose of breadcrumbs

				return path_sequence;
			}
		}

		public static double[][] waypointSequence(List<Long> path_sequence)
		{
			synchronized (crumbs_lock)
			{
				double[][] waypoints = new double[path_sequence.size()][2];
				for (int i = 0; i < path_sequence.size(); i++) {
					UTM utm = crumbs_by_index.get(path_sequence.get(i)).getLocation();
					LatLong latlong = UTM.utmToLatLong(utm, ReferenceEllipsoid.WGS84);
					waypoints[i] = new double[]{latlong.latitudeValue(NonSI.DEGREE_ANGLE), latlong.longitudeValue(NonSI.DEGREE_ANGLE)};
				}
				return waypoints;
			}
		}
}