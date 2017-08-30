package com.platypus.android.server;

import android.os.Environment;
import android.util.Log;

import com.platypus.crw.data.UtmPose;

import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;
import org.json.JSONObject;
import org.json.JSONTokener;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import java.util.Scanner;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.ScheduledThreadPoolExecutor;

import java.util.function.Predicate;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.measure.unit.NonSI;
import javax.measure.unit.SI;


/**
 * Created by jason on 8/4/17.
 *
 * https://docs.oracle.com/javase/tutorial/java/javaOO/lambdaexpressions.html
 * https://docs.oracle.com/javase/8/docs/api/java/util/function/package-summary.html
 * https://developer.android.com/studio/preview/install-preview.html
 * http://www.java2s.com/Tutorials/Java_Lambda/java.util.function/BooleanSupplier/BooleanSupplier_example.htm
 * "currying"
 * https://developer.android.com/reference/org/json/JSONTokener.html
 * http://regexr.com/
 * http://www.regexplanet.com/advanced/java/index.html
 */

public class AutonomousPredicates
{
		VehicleServerImpl _serverImpl;
		String logTag = "AP";
		static long ap_count = 0;
		Map<Long, ScheduledFuture> triggered_actions_map = new HashMap<>();
		ScheduledThreadPoolExecutor poolExecutor = new ScheduledThreadPoolExecutor(4);
		final double ISNEAR_DISTANCE_THRESHOLD = 3;
		// TODO: do we want something that can increase and decrease the thread pool, rather than fixed?

		class TriggeredAction implements Runnable
		{
				long _id;
				boolean _isPermanent; // if true, this triggered action Runnable will be canceled after it runs once
				Predicate<Void> _test; // meant to call methods in the VehicleServerImpl
				String _action; // meant to call methods in the VehicleServerImpl
				String _name;

				public TriggeredAction(String name, Predicate<Void> test, String action, boolean isPermanent)
				{
						_id = ap_count++;
						_isPermanent = isPermanent;
						_test = test;
						_action = action;
						_name = name;
				}

				public long getID() { return _id; }

				@Override
				public void run()
				{
						Log.d(logTag, String.format("Task %s running...", _name));
						if (_test.test(null))
						{
								Log.i(logTag, String.format("Task %s test returned TRUE, executing task...", _name));
								_serverImpl.performAction(_action);
								if (!_isPermanent)
								{
										Log.i(logTag, String.format("Task %s completed, removing...", _name));
										triggered_actions_map.get(_id).cancel(true);
										triggered_actions_map.remove(_id);
								}
						}
						else
						{
								Log.i(logTag, String.format("Task %s test returned FALSE", _name));
						}
				}
		}

		class DynamicPredicateComposition
		{
				/*
				VERY IMPORTANT NOTE
				Predicates chained with and() and or() short circuit (i.e. skip evaluating subsequent
				predicates).
				From the Java docs for and():
					Returns a composed predicate that represents a short-circuiting logical AND of this
					predicate and another. When evaluating the composed predicate, if this predicate is
					false, then the other predicate is not evaluated.
				From the Java docs for or():
					Returns a composed predicate that represents a short-circuiting logical OR of this
					predicate and another. When evaluating the composed predicate, if this predicate is
					true, then the other predicate is not evaluated.
				Because of this, the first predicate in the chain is ALWAYS evaluated.
			  Note that something.or(other) short circuits if *something* is true, NOT if *other* is true.

				true/false && {only matters if other thing is true}
				true/false || {only matters if other thing is false}

				Supports complex predicates, such as conjunctive normal form: (A or B or C) AND (C or D or E)

			  ANOTHER VERY IMPORTANT NOTE
			  The "builder pattern" is used here, and that has some consequences.
			  Each call to and() or or() on an individual DynamicPredicateComposition instance enforces
			    nested parenthesis!
			  For example dpc.or(test 1).or(test 2).and(test 3) is equivalent to
			    (test 1 or test 2) and test 3
			  If you want parenthesis to appear around test 2 and test 3 instead (which definitely changes
			    outcome of the test!), you need to use more than one DynamicPredicateComposition instance.
			  dpc1.or(test 1);
			  dpc2.or(test 2).and(test 3);
			  dpc1.build().or(dpc2.build()); --> this is now equivalent to test 1 or (test 2 and test 3)
				*/
				int depth = 0; // can serve as unique ID for the predicates
				Predicate<Void> predicate = new Predicate<Void>()
				{
						@Override
						public boolean test(Void v)
						{
								// Log.v(logTag, "Executing the original default predicate");
								return false; // must start as false and composition must start with an OR
						}
				};
				public Predicate<Void> build()
				{
						Log.i(logTag, "Building composite predicate...");
						return predicate;
				}
				private Predicate<Void> generatePredicate(final String left_hand_side, final String comparator, final double right_hand_side) throws Exception
				{
						final String definition = String.format("%s %s %f", left_hand_side, comparator, right_hand_side);

						Log.d(logTag, String.format("Generating new predicate: %s", definition));

						return new Predicate<Void>()
						{
								@Override
								public boolean test(Void v) // the input to this is never used
								{
										Object retrieval = null;
										try
										{
												retrieval = _serverImpl.getState(left_hand_side);
										}
										catch (Exception e)
										{
												Log.e(logTag, e.getMessage());
										}
										if (retrieval == null)
										{
												Log.w(logTag, String.format("Predicate %s received a null when it asked for %s", definition, left_hand_side));
												return false;
										}
										Double a = Double.class.cast(retrieval);
										Double b = Double.class.cast(right_hand_side);
										boolean result = false;
										switch(comparator)
										{
												case "=":
												case "==":
														result = a == b;
														break;
												case "!=":
														result = a != b;
														break;
												case "<":
														result = a < b;
														break;
												case "<=":
														result = a <= b;
														break;
												case ">":
														result = a > b;
														break;
												case ">=":
														result = a >= b;
														break;
												default:
														break;
										}
										Log.d(logTag, String.format("Executed a predicate: %s is %s", definition, Boolean.toString(result)));
										return result;
								}
						};
				}
				private Predicate<Void> generatePredicate(final String boolean_state) throws Exception
				{
						//String my_boolean_state = boolean_state;
						Log.d(logTag, String.format("Generating new boolean only predicate"));
						/*
						// look for leading "^" i.e. a NOT. If present, trim it and negate the generated predicate
						Pattern not_pattern = Pattern.compile("[\\^]");
						Matcher not_matcher = not_pattern.matcher(boolean_state);
						final boolean negated = not_matcher.find();
						if (negated)
						{
								Pattern the_rest = Pattern.compile("[^\\^]+");
								Matcher the_rest_matcher = the_rest.matcher(boolean_state);
								the_rest_matcher.find();
								my_boolean_state = the_rest_matcher.group().trim();
								Log.d(logTag, String.format("Negating boolean only predicate %s", my_boolean_state));
						}

						final String boolean_state_final = my_boolean_state;
						// if "^" character is present, use Predicate.negate() to return a negated predicate
						*/


						return new Predicate<Void>()
						{
								@Override
								public boolean test(Void v) // the input to this is never used
								{
										try
										{
												Object retrieval = _serverImpl.getState(boolean_state);
												if (retrieval == null)
												{
														Log.w(logTag, String.format("Boolean only predicate received a null when it asked for %s", boolean_state));
														return false;
												}
												Boolean result = Boolean.class.cast(retrieval);
												/*
												if (negated)
												{
														Log.d(logTag, String.format("Executed a predicate: ^%s = %s", boolean_state_final, Boolean.toString(!result)));
												}
												else
												{
														Log.d(logTag, String.format("Executed a predicate: %s = %s", boolean_state_final, Boolean.toString(result)));
												}
												*/
												Log.d(logTag, String.format("Executed a predicate: %s = %s", boolean_state, Boolean.toString(result)));
												return result;
										}
										catch (Exception e)
										{
												Log.e(logTag, String.format("Boolean only predicate error: %s", e.getMessage()));
												return false;
										}
								}
						};
				}

				public DynamicPredicateComposition and(final Predicate<Void> _predicate) throws Exception
				{
						depth++;
						if (_predicate == null) throw new Exception("Input predicate is null");
						if (depth == 1)
						{
								depth = 0;
								return or(_predicate);
						}
						predicate = predicate.and(_predicate);
						Log.d(logTag, "Added a compound AND");
						return this;
				}
				public DynamicPredicateComposition and(final String left_hand_side, final String comparator, final double right_hand_side) throws Exception
				{
						depth++;
						if (depth == 1) // MUST START WITH AN OR
						{
								depth = 0;
								return or(left_hand_side, comparator, right_hand_side);
						}
						predicate = predicate.and(generatePredicate(left_hand_side, comparator, right_hand_side));
						Log.d(logTag, String.format("Added an AND: %s %s %.0f", left_hand_side, comparator, right_hand_side));
						return this;
				}
				public DynamicPredicateComposition and(final String boolean_state) throws Exception
				{
						depth++;
						if (depth == 1) // MUST START WITH AN OR
						{
								depth = 0;
								return or(boolean_state);
						}
						predicate = predicate.and(generatePredicate(boolean_state));
						Log.d(logTag, String.format("Added an AND: %s", boolean_state));
						return this;
				}
				public DynamicPredicateComposition or(final Predicate<Void> _predicate) throws Exception
				{
						depth++;
						if (_predicate == null) throw new Exception("Input predicate is null");
						predicate = predicate.or(_predicate);
						Log.d(logTag, "Added a compound OR");
						return this;
				}
				public DynamicPredicateComposition or(final String left_hand_side, final String comparator, final double right_hand_side) throws Exception
				{
						depth++;
						predicate = predicate.or(generatePredicate(left_hand_side, comparator, right_hand_side));
						Log.d(logTag, String.format("Added an OR: %s %s %.0f", left_hand_side, comparator, right_hand_side));
						return this;
				}
				public DynamicPredicateComposition or(final String boolean_state) throws Exception
				{
						depth++;
						predicate = predicate.or(generatePredicate(boolean_state));
						Log.d(logTag, String.format("Added an OR: %s", boolean_state));
						return this;
				}

				public Predicate<Void> inInterval(final String left_hand_side, final double low, final double high) throws Exception
				{
						Predicate<Void> new_predicate = generatePredicate(left_hand_side, ">=", low);
						new_predicate = new_predicate.and(generatePredicate(left_hand_side, "<=", high));
						return new_predicate;
				}

				public Predicate<Void> isNear(final double latitude, final double longitude, final double radius)
				{
						Predicate<Void> new_predicate = new Predicate<Void>()
						{
								final UTM location_utm = UTM.latLongToUtm(LatLong.valueOf(latitude, longitude, NonSI.DEGREE_ANGLE), ReferenceEllipsoid.WGS84);
								@Override
								public boolean test(Void aVoid)
								{
										// calculate distance in meters away from location_utm
										double distance = -1;
										boolean result = false;
										try
										{
												UtmPose utmPose = _serverImpl.getState(VehicleState.States.CURRENT_POSE.name);
												distance = Math.sqrt(Math.pow(utmPose.pose.getX() - location_utm.eastingValue(SI.METER), 2.0)
																+ Math.pow(utmPose.pose.getY() - location_utm.northingValue(SI.METER), 2.0));
												result = distance < radius;
										}
										catch (Exception e)
										{
												Log.e(logTag, e.getMessage());
										}
										Log.d(logTag, String.format("Executed isNear predicate: Distance from target %f < %f is %s", distance, radius, Boolean.toString(result)));
										return result;
								}
						};
						return new_predicate;
				}

				// TODO: useful building utilities
				/*
				List of ideas:
				1) inConvexHull(vertices[]) --> current location is inside polygon defined by these
				*/
		}

		public void loadDefaults()
		{
				loadFromFile("default_behaviors.txt");
		}
		private void loadFromFile(String filename)
		{
				final File file = new File(Environment.getExternalStorageDirectory() + "/platypus_behaviors/" + filename);
				/*
				1) read all lines in the human readable file, put them into a single string
				2) JSONTokener parses human readable string into a JSONObject with all default behaviors
				3) Split up the one large JSONObject and parse each task/behavior
				*/

				Scanner fileScanner;
				try
				{
						fileScanner = new Scanner(file);
				}
				catch (Exception e)
				{
						Log.e(logTag, e.getMessage());
						return;
				}

				// gather all the default behaviors
				StringBuffer buffer = new StringBuffer();
				JSONObject[] tasks = null;
				if (file.exists())
				{
						while (fileScanner.hasNext())
						{
								buffer.append(fileScanner.nextLine());
						}
				}
				String human_string = buffer.toString();
				JSONTokener tokener = new JSONTokener(human_string);
				JSONObject file_json;
				try
				{
						file_json = (JSONObject)tokener.nextValue();
						// Log.v(logTag, file_json.toString(2));
				}
				catch (Exception e)
				{
						Log.e(logTag, e.getMessage());
						return;
				}

				// parse each behavior and generate tasks
				Iterator<String> file_keys = file_json.keys();
				String key;
				while (file_keys.hasNext())
				{
						key = file_keys.next();
						Log.i(logTag, String.format("Next task: %s", key));
						try
						{
								JSONObject task_json = (JSONObject)file_json.get(key);
								// Log.v(logTag, task_json.toString(2));
								createTask(task_json, key);
						}
						catch (Exception e)
						{
								Log.e(logTag, e.getMessage());
								continue;
						}
				}
		}

		private Predicate<Void> parseTrigger(String predicate_string)
		{
				// Split the trigger string and create compound predicate from it

				DynamicPredicateComposition dpc = new DynamicPredicateComposition();

				String boolean_regex = "[|&]+(?![^\\(]*\\))"; // split on boolean logic symbols, but don't split up parentheses
				String[] predicate_strings = predicate_string.split(boolean_regex);
				Log.d(logTag, String.format("predicates: %s", Arrays.toString(predicate_strings)));

				/*
				Each call of parseTrigger should have its own boolean list.
				Only the booleans outside of compound predicates should be included.
				Because the compound predicates will get their own parseTrigger call.
				*/

				Pattern boolean_pattern = Pattern.compile(boolean_regex);
				Matcher boolean_matcher = boolean_pattern.matcher(predicate_string);
				List<String> booleans_list = new ArrayList<>();
				while (boolean_matcher.find()) booleans_list.add(boolean_matcher.group());
				Log.v(logTag, String.format("booleans: %s", Arrays.toString(booleans_list.toArray())));
				booleans_list.add(0, "|"); // include an extra leading OR symbol "|"

				Pattern parenthesis_pattern = Pattern.compile("[()]+"); // used to find any parentheses easily
				Pattern leading_negation_pattern = Pattern.compile("^\\^"); // MUST be used on trimmed string (a leading space ruins it)
				Pattern inner_pattern = Pattern.compile("(?:\\([^()]+\\)|[^()])+(?=\\))"); // only stuff inside the outermost parentheses
				String predicate_symbol_regex = "[[<>!=]=?:@]+"; // split on predicate symbols. Account for possibility of >=, <=, ==, and != as individual symbols.
				int predicate_count = 0;
				try
				{
						for (String predicate : predicate_strings)
						{
								predicate = predicate.trim(); // MUST eliminate any leading spaces!
								String splitting_boolean = booleans_list.get(predicate_count);
								Log.v(logTag, String.format("Using splitting boolean %s", splitting_boolean));
								Matcher parenthesis_matcher = parenthesis_pattern.matcher(predicate);
								if (parenthesis_matcher.find())
								{
										// contains parentheses. Compound predicate. Trim off outer parentheses and recurse.
										// TODO: look for a leading "^" that negates the parentheses
										Log.v(logTag, String.format("predicate %s is compound. Need to recurse", predicate));
										Matcher inner_matcher = inner_pattern.matcher(predicate);
										if (inner_matcher.find())
										{
												String inner_predicate_string = inner_matcher.group();
												Predicate<Void> inner_predicate = parseTrigger(inner_predicate_string);
												// TODO: use inner_predicate.negate() if there was a leading "^"
												Matcher negation_matcher = leading_negation_pattern.matcher(predicate);
												if (negation_matcher.find())
												{
														Log.d(logTag, String.format("Predicate %s will be negated", inner_predicate_string));
														inner_predicate = inner_predicate.negate();
												}
												Log.d(logTag, String.format("Using splitting boolean %s on compound predicate", splitting_boolean));
												// based on splitting boolean, call DynamicPredicateComposition methods using the above dpc object
												switch (splitting_boolean)
												{
														case "&":
																dpc.and(inner_predicate);
																break;
														case "|":
																dpc.or(inner_predicate);
																break;
														default:
																Log.e(logTag, "Unknown boolean, skipping");
																break;
												}
										}
								}
								else
								{
										String[] components = predicate.split(predicate_symbol_regex);
										// trim extra spaces from all the components
										for (int i = 0; i < components.length; i++)
										{
												components[i] = components[i].trim();
										}

										Log.v(logTag, String.format("%s --> components: %s", predicate, Arrays.toString(components)));

										// based on symbol and splitting boolean, call DynamicPredicateComposition methods using the above dpc object
										// If there is no symbol, it is a pure boolean predicate
										// If it is ":", the first component must be within the interval (inclusive) of the two values given in second component
										// If it is "@", the first component must be within a euclidean distance of the second component
										// If it is ">, <, >=, =, !=, the first component is compared against the second component
										// If there is a leading "^", this is a NOT. Only applicable to pure boolean predicates.
										Pattern symbol_pattern = Pattern.compile(predicate_symbol_regex);
										Matcher symbol_matcher = symbol_pattern.matcher(predicate);
										if (symbol_matcher.find())
										{
												String symbol = symbol_matcher.group();
												Log.v(logTag, String.format("Using predicate symbol \"%s\"", symbol));
												switch (symbol)
												{
														case ":":
																// retrieve the numbers inside the interval
																Pattern number_pattern = Pattern.compile("[+-]?([0-9]*[.])?[0-9]+"); // any integer and floating point numbers
																Matcher number_matcher = number_pattern.matcher(components[1]);
																number_matcher.find();
																String low_string = number_matcher.group();
																number_matcher.find();
																String high_string = number_matcher.group();
																double low = Double.valueOf(low_string);
																double high = Double.valueOf(high_string);
																Log.v(logTag, String.format("Using [low, high] = %f, %f", low, high));
																switch (splitting_boolean)
																{
																		case "&":
																				dpc.and(dpc.inInterval(components[0], low, high));
																				break;
																		case "|":
																				dpc.or(dpc.inInterval(components[0], low, high));
																				break;
																		default:
																				break;
																}
																break;
														case "@":
																number_pattern = Pattern.compile("[+-]?([0-9]*[.])?[0-9]+"); // any integer and floating point numbers
																number_matcher = number_pattern.matcher(components[1]);
																number_matcher.find();
																String lat_string = number_matcher.group();
																number_matcher.find();
																String lon_string = number_matcher.group();
																double lat = Double.valueOf(lat_string);
																double lon = Double.valueOf(lon_string);
																Log.d(logTag, String.format("Using [lat, long] = %f, %f", lat, lon));
																switch (splitting_boolean)
																{
																		case "&":
																				dpc.and(dpc.isNear(lat, lon, ISNEAR_DISTANCE_THRESHOLD));
																				break;
																		case "|":
																				dpc.or(dpc.isNear(lat, lon, ISNEAR_DISTANCE_THRESHOLD));
																				break;
																		default:
																				break;
																}
																break;
														case "<":
														case "<=":
														case "=":
														case "==":
														case "!=":
														case ">=":
														case ">":
																switch (splitting_boolean)
																{
																		case "&":
																				dpc.and(components[0], symbol, Double.valueOf(components[1]));
																				break;
																		case "|":
																				dpc.or(components[0], symbol, Double.valueOf(components[1]));
																				break;
																		default:
																				break;
																}
																break;
														default:
																break;
												}
										}
										else
										{
												// pure boolean predicate
												switch (splitting_boolean)
												{
														case "&":
																dpc.and(components[0]);
																break;
														case "|":
																dpc.or(components[0]);
																break;
														default:
																break;
												}
										}
								}
								predicate_count++;
						}
						return dpc.build();
				}
				catch (Exception e)
				{
						Log.e(logTag, String.format("Could not parse trigger. Error = %s", e.getMessage()));
						return null;
				}
		}

		private void createTask(JSONObject definition, String name)
		{
				String key;
				Iterator<String> task_keys = definition.keys();

				String action = new String();
				long ms_interval = 1000;
				boolean ends = true;
				Predicate<Void> predicate = null;
				try
				{
						while (task_keys.hasNext())
						{
								key = task_keys.next();
								// Log.v(logTag, String.format("Next task key: %s", key));
								switch(key)
								{
										case "action":
												action = definition.getString(key);
												action = action.trim();
												// make sure the action is one of the available ones
												if (!VehicleServerImpl.Actions.contains(action)) throw new Exception("task definition contains unknown action");
												break;
										case "trigger":
												predicate = parseTrigger(definition.getString(key));
												break;
										case "interval":
												ms_interval = definition.getLong(key);
												break;
										case "ends":
												String ends_string = (definition.getString(key)).toLowerCase();
												switch(ends_string)
												{
														case "y":
														case "yes":
														case "true":
																ends = true;
																break;
														case "n":
														case "no":
														case "false":
																ends = false;
																break;
														default:
																throw new Exception("task definition \"ends\" field must be y or n");
												}
												break;
										default:
												break;
								}
						}

						if (predicate == null)
						{
								Log.e(logTag, "Predicate was null. Not scheduling a task.");
								return;
						}

						// create new triggered action
						// Need Predicate, string for action,
						TriggeredAction ta = new TriggeredAction(name, predicate, action, !ends);

						// put the triggered action task into the scheduler queue and store its ScheduledFuture in the HashMap (so we can cancel it later)
						triggered_actions_map.put(ta.getID(), poolExecutor.scheduleAtFixedRate(ta, 0, ms_interval, TimeUnit.MILLISECONDS));
				}
				catch (Exception e)
				{
						Log.e(logTag, e.getMessage());
				}
		}

		public AutonomousPredicates(VehicleServerImpl server)
		{
				_serverImpl = server;
				Log.w(logTag, "**** AutonomousPredicates constructor ****");
		}

		public void cancelAll()
		{
				for (Map.Entry<Long, ScheduledFuture> entry : triggered_actions_map.entrySet())
				{
						entry.getValue().cancel(true);
				}
				triggered_actions_map.clear();
				ap_count = 0;
		}

		void displayActions()
		{
				// display some kind of summary of the current trigger definitions in the debug activity
		}
}
