import networkx as nx
import time
import itertools

"""
	Functions for traversing a network graph.
	
	Generally the approach is to create a path
	through connected edges that is as long as possible.


"""
def pairwise(iterable):
	"s -> (s0,s1), (s1,s2), (s2, s3), ..."
	a, b = itertools.tee(iterable)
	next(b, None)
	return itertools.izip(a, b)
	

def isFillEdge(graph,n1,n2):
	return graph[n1][n2]['type'] == 'F';

def isUsed(graph, n1, n2):
	return graph[n1][n2].has_key('used');


def markUsed(graph, n1, n2):
	graph[n1][n2]['used'] = True;

def getNextEdge(graph, edgeList):
	for e in edgeList:
		if not isUsed(graph,e[0],e[1]):
			return e;
	return None;

def travelAll(graph,edgeList,fillFunction=isFillEdge ):
	"""
		The goal here is to find a path, if possible, that will chain
		all of the requested edges together along borders.
		
		the logic we want is:
			start with the first edge in the list
			draw all edges in the list
			connect edges with the shortest possible path
			preferring afer that the order provided in the list
	"""
	edgesToDraw = list(edgeList); #a copy
	allPaths = []
	nextFillEdge = getNextEdge(graph,edgesToDraw);

	while nextFillEdge != None:
		path = findPath(graph, [ nextFillEdge[0],nextFillEdge[1] ],fillFunction);
		allPaths.append(path);		
		nextFillEdge = getNextEdge(graph,edgesToDraw);		
	return allPaths;

def findPath(graph, path,fillFunction=isFillEdge):
	"""
		starting with an existing path, continue until it is not possible to reach an infill edge
		
		RULES:
			cannot go through a node more than once
			cannot trace any edge more than once
			
	"""
	#mark any part of the path coming in used
	#hack-- i hate that 
	for (p1,p2) in pairwise(path):
		markUsed(graph,p1,p2);
		
	s = path[-1];
	searchChoices = [];
	
	for n in graph.neighbors(s):
		if n in path:
			continue;
		if isUsed(graph,n,s): 
			continue;
		if fillFunction(graph,n,s):
			return findPath(graph,path + [n],fillFunction);
		else:
			searchChoices.append([s,n]);
			
	#ok if we get here, we evaluate travelling on boundaries
	#we need to do a breadth first search till we get to a fill edge
	visited = set(s);
	while len(searchChoices) > 0:
		cp = searchChoices.pop(0);
		(n1,n2) = (cp[-2],cp[-1]);
		if isUsed(graph,n1,n2): 
			continue; #skip any used edges
		if fillFunction(graph,n1,n2):
			return findPath(graph,path + cp[1:],fillFunction); #this feels wrong!
		
		#else, search other paths-- they are boundaries
		visited.add(n2);
		visited.add(n1);
		for p in graph.neighbors(n2):
			if p in visited: 
				continue;
			else:
				searchChoices.append(cp + [p]);
	
	#ok if we get here, there are no more paths to search, so we exit
	return path;
		

def assertPathListEquals(expected, actual):
	"""
		makes it easier to assert path traversal.
		expected is in the form of a list of strings, such as
		[ 'ABCD', 'EFGH', 'HIJK' ] -- each string of letters is a chain of nodes.
	"""
	if len(expected) != len(actual):
		raise AssertionError("Lenths do not match. Got '%s', expected '%s'" % (str(actual), str(expected) ) )
	
	for (e,a) in zip(expected,actual):
		#e is a string of characters, a is a list of actual nodes
		assertSinglePathEquals(e,a);
			
def assertSinglePathEquals(expected, acutal):
		path = "".join(acutal);
		assert expected == path, "Expected %s, Got %s " % ( expected, acutal)
		
if __name__=='__main__':
	g = nx.Graph()
	fill_edges = [('D','M'),('M','B'),('J','C'),('G','I'),('F','K')]
	bound_edges = [('A','B'),('B','C'),('C','E'),('E','F'),('F','G'),('G','H'),('H','I'),('I','K'),('K','L'),('L','J'),('D','J'),('D','A')  ]; #boundary
	

	for f in fill_edges:
		g.add_edge(f[0],f[1],type='F' );
	for f in bound_edges:
		g.add_edge(f[0],f[1],type='B' );
	
	#note: syntax is a little funny. each expected path is a list of paths. pathToNextFill returns a single path,
	#but walkAllInFill returns a list of paths.
	#assertPathEquals is designed to test the results of walkInfill
	q = time.clock();	

	#tests for finding single paths
	assertSinglePathEquals('DMBCJLKFGI', findPath(g.copy(), ['D','M'])); #should walk then entire graph	
	assertSinglePathEquals('JCBMD', findPath(g.copy(), ['J','C'])) #should get stuck, cannot traverse node J again	
	assertSinglePathEquals('ADMBCJLKFGI', findPath(g.copy(), ['A','D','M','B'])) #should preserve first part of the path correctly
	assertSinglePathEquals('ADMBCJLKFGI', findPath(g.copy(), ['A','D'])) #should work even if first path is not a fill edge
	assertSinglePathEquals('ABMDJCEFKIG', findPath(g.copy(), ['A'])) #should work even if first path is a single node
	
	#tests for walking through a list of paths
	assertPathListEquals(['DMBCJLKFGI'],travelAll(g.copy(),fill_edges));
	
	fill_edges2=[ ('J','C'),('B','M'),('M','D' ),('G','I'),('K','F') ]; #note JC is first-- should result in two paths	
	assertPathListEquals(['JCBMD','GIKF'],travelAll(g.copy(),fill_edges2));
	
	fill_edges3=[ ('J','C'),('K','F'),('B','M'),('M','D' ),('G','I') ]; #note different order	
	assertPathListEquals(['JCBMD','KFGI'],travelAll(g.copy(),fill_edges3));
	
	print "%0.3f elapsed" % ( time.clock() -  q );