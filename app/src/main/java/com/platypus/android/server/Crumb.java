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
import java.util.List;
import java.util.Map;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Created by jason on 6/7/17.
 */

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
		private void setParent(long _parent) { parent = _parent; }
		private long getParent() { return parent; }
		private double getCost() { return g + h; }
		long getIndex() { return index; }
		UTM getLocation() { return location; }
		public List<Long> getSuccessors() { return successors; }

		// Static fields
		private final static double MAX_NEIGHBOR_DISTANCE = 5;
		private static Map<Long, Crumb> crumbs_by_index = new HashMap<>();
		private static Map<Long, Crumb> unsent_crumbs = new HashMap<>();
		private static Map<Long, Map<Long, Double>> pairwise_distances = new HashMap<>();
		private static Map<Long, List<Long>> neighbors = new HashMap<>();
		private static final Object crumbs_lock = new Object();
		private static String logTag = "crumbs";
		static void checkForNewCrumb(UtmPose current_utmpose)
		{

			// TODO: Need to check how I can intelligently throw away old or redundant crumbs to limit the total number
			// TODO: Perhaps I could loop through the pairwise distances and remove ones that are within the hull of their neighbors?
			// TODO: Or maybe there is a way of hashing a crumb by the sorted indices of its neighbors?
			// TODO: So if there are crumbs that share all the same neighbors, eliminate one of them
			synchronized (crumbs_lock) {
				if (current_utmpose.isDefault()) return; // ignore default location
				long last_index = crumbs_by_index.size();
				if (last_index < 1) {
					Log.i(logTag, "Generating first crumb");
					newCrumb(UTM.valueOf(
							current_utmpose.origin.zone,
							current_utmpose.origin.isNorth ? 'T' : 'L',
							current_utmpose.pose.getX(),
							current_utmpose.pose.getY(),
							SI.METER
					));
					return;
				}
				Log.v(logTag, String.format("Checking to drop a new crumb..."));
				Crumb last_crumb = crumbs_by_index.get(last_index - 1);
				UTM last_utm = last_crumb.getLocation();
				double distance = Math.pow(current_utmpose.pose.getX() - last_utm.eastingValue(SI.METER), 2.0);
				distance += Math.pow(current_utmpose.pose.getY() - last_utm.northingValue(SI.METER), 2.0);
				if (distance >= 0.5 * MAX_NEIGHBOR_DISTANCE * MAX_NEIGHBOR_DISTANCE) {
					// note the 0.5 coefficient. This ensures the crumbs are generated a little closer than the cutoff for neighbors
					Log.i(logTag, "Generating a new crumb");
					newCrumb(UTM.valueOf(
							current_utmpose.origin.zone,
							current_utmpose.origin.isNorth ? 'T' : 'L',
							current_utmpose.pose.getX(),
							current_utmpose.pose.getY(),
							SI.METER
					));
				}
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
						return unsent_crumbs.get(index);
				}
		}

		static void acknowledge(long _id)
		{
				synchronized (crumbs_lock)
				{
						if (unsent_crumbs.containsKey(_id)) unsent_crumbs.remove(_id);
				}
		}

		// Static methods
		private static double distanceBetweenUTM(UTM location_i, UTM location_j)
		{
			double dx = (location_i.eastingValue(SI.METER) - location_j.eastingValue(SI.METER));
			double dy = (location_i.northingValue(SI.METER) - location_j.northingValue(SI.METER));
			return Math.sqrt(dx*dx + dy*dy);
		}

		private static double distanceBetweenCrumbs(long index_i, long index_j)
		{
				UTM location_i = crumbs_by_index.get(index_i).getLocation();
				UTM location_j = crumbs_by_index.get(index_j).getLocation();
				return distanceBetweenUTM(location_i, location_j);
		}

		private static long newCrumb(UTM _location)
		{
			// TODO: make sure this is all threadsafe
			// TODO: since we want to turn off crumb generation while going home anyway, we can accept the large blocking delay if we synchronize the entirety of aStar
			// TODO: IMPORTANT - do *not* synchronize newCrumb itself. I don't know if synchronization is recursive, if not then we have immediate deadlock when aStar calls newCrumb!
			// TODO: 		so, is java synchronization recursive? i.e. can a synchronized block call a function with its own synchronized block, both using the same lock object?
			// TODO:        Answer: https://stackoverflow.com/questions/13197756/synchronized-method-calls-itself-recursively-is-this-broken
			// TODO:		In java, synchronization is re-entrant within a thread. Okay, so hypothetically I could synchronize newCrumb() as well.
			// TODO:		But I don't see why I would.
				try
				{
					// initialize objects
					long new_index = crumbs_by_index.size();
					Crumb new_crumb = new Crumb(new_index, _location);
					crumbs_by_index.put(new_index, new_crumb);
					unsent_crumbs.put(new_index, new_crumb);
					pairwise_distances.put(new_index, new HashMap<Long, Double>());
					neighbors.put(new_index, new ArrayList<Long>());

					// calculate pairwise distances and neighbors
					for (Map.Entry<Long, Crumb> entry_i : crumbs_by_index.entrySet())
					{
							long index_i = entry_i.getKey();
							for (Map.Entry<Long, Crumb> entry_j : crumbs_by_index.entrySet())
							{
									long index_j = entry_j.getKey();

									// if a Crumb is being compared to itself
									// OR
									// if a calculation was previously performed for pair (i,j)
									if (index_i == index_j || pairwise_distances.get(index_i).containsKey(index_j))
									{
											continue; // don't perform the calculations
									}

									double pairwise_distance = distanceBetweenCrumbs(index_i, index_j);
									pairwise_distances.get(index_i).put(index_j, pairwise_distance);
									pairwise_distances.get(index_j).put(index_i, pairwise_distance);
									if (pairwise_distance <= MAX_NEIGHBOR_DISTANCE)
									{
											neighbors.get(index_i).add(index_j);
											neighbors.get(index_j).add(index_i);
									}
							}
					}
					return new_index;
				}
				catch (Exception e)
				{
						Log.e("crumbs", String.format("newCrumb() error: %s", e.getMessage()));
						return -1;
				}
		}

		public static List<Long> straightHome(UTM start, UTM goal)
		{
			synchronized (crumbs_lock)
			{
				// Simple: go straight home from the start
				List<Long> path_sequence = new ArrayList<>();
				long start_index = newCrumb(start);
				long goal_index = newCrumb(goal);
				path_sequence.add(start_index);
				path_sequence.add(goal_index);
				return path_sequence;
			}
		}

		public static List<Long> aStar(UTM start, UTM goal)
		{
			synchronized (crumbs_lock)
			{
				// TODO: I need to stop new crumbs from being created while the boat is going home!
				Log.i("aStar", "Starting A* calculation...");
				long start_index = newCrumb(start);
				long goal_index = newCrumb(goal);

				// TODO: force the start to be reachable -- this should be guaranteed if we use current location as the start

				// make sure goal is reachable (i.e. it has at least one neighbor, otherwise use crumb closest to goal as the new goal
				// use a while loop, but make sure that it doesn't loop endlessly
				int max_attempts = crumbs_by_index.size();
				int attempt_count = 0;
				boolean goal_is_unreachable;
				do {
					attempt_count += 1;
					Log.i("aStar", String.format("start_index = %d, goal_index = %d", start_index, goal_index));
					goal_is_unreachable = neighbors.get(goal_index).isEmpty();
					if (goal_is_unreachable) Log.w("aStar", "Goal is unreachable, using closest crumb as new goal");

					// fill in distance to goal values
					double min_dist_to_goal = 99999999;
					long new_goal_index = -1;
					for (Map.Entry<Long, Crumb> entry : crumbs_by_index.entrySet())
					{
						double dist_to_goal = distanceBetweenCrumbs(entry.getKey(), goal_index);
						entry.getValue().setH(dist_to_goal);
						if (goal_is_unreachable && entry.getKey() != goal_index)
						{
							if (dist_to_goal < min_dist_to_goal)
							{
								min_dist_to_goal = dist_to_goal;
								new_goal_index = entry.getKey();
							}
						}
						// verbose level print all info about all crumbs
						String crumb_info = String.format("crumb #%d, neighbors:[", entry.getKey());
						for (long neighbor : neighbors.get(entry.getKey()))
						{
							crumb_info += String.format("%d, ", neighbor);
						}
						crumb_info += "]";
						Log.w("aStar", crumb_info);
					}
					if (goal_is_unreachable) goal_index = new_goal_index;
				} while (goal_is_unreachable && attempt_count < max_attempts);

				HashMap<Long, Void> open_crumbs = new HashMap<>();
				HashMap<Long, Double> open_costs = new HashMap<>();
				HashMap<Long, Void> closed_crumbs = new HashMap<>();
				List<Long> path_sequence = new ArrayList<>();
				long current_crumb = 0;
				long iterations = 0;
				open_crumbs.put(start_index, null);
				Log.d("aStar", String.format("open_crumbs.size() = %d", open_crumbs.size()));

				while (open_crumbs.size() > 0) {
					iterations += 1;
					Log.d("aStar", String.format("A* iter %d:  %d open, %d closed",
							iterations, open_crumbs.size(), closed_crumbs.size()));

					// recreate open costs map
					open_costs.clear();
					double lowest_cost = 99999999;
					for (Map.Entry<Long, Void> entry : open_crumbs.entrySet()) {
						double cost = crumbs_by_index.get(entry.getKey()).getCost();
						open_costs.put(entry.getKey(), cost);
						if (cost < lowest_cost) {
							//Log.v("aStar", String.format("crumb # %d has lowest cost = %.2f", entry.getKey(), cost));
							lowest_cost = cost;
							current_crumb = entry.getKey();
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
						if (open_crumbs.containsKey(s)) {
							if (crumbs_by_index.get(s).getG() <= potential_g) continue;
						} else if (closed_crumbs.containsKey(s)) {
							if (crumbs_by_index.get(s).getG() > potential_g) {
								open_crumbs.put(s, null);
								closed_crumbs.remove(s);
							} else {
								continue;
							}
						} else {
							open_crumbs.put(s, null);
						}
						crumbs_by_index.get(s).setG(potential_g);
						crumbs_by_index.get(s).setParent(current_crumb);
					}
					open_crumbs.remove(current_crumb);
					closed_crumbs.put(current_crumb, null);
				}
				path_sequence.add(current_crumb);
				while (path_sequence.get(0) != start_index) {
					path_sequence.add(0, crumbs_by_index.get(path_sequence.get(0)).getParent());
				}
				Log.i("aStar", "Path sequence = " + path_sequence.toString());
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