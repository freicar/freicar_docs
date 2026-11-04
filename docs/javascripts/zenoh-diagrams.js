/* Rounded upward-then-horizontal client links for tracking-only diagrams. */
(() => {
  function routeTrackingClients() {
    document.querySelectorAll('.zenoh-tracking svg').forEach((svg) => {
      if (svg.dataset.clientsRouted) return;
      const base = svg.querySelector('g.node[id="B"]');
      const clients = [...svg.querySelectorAll('path.flowchart-link[id*="-S"][id$="-B"]')];
      if (!base || clients.length !== 4) return;

      function bounds(node) {
        const box = node.getBBox();
        const matrix = node.transform.baseVal.consolidate().matrix;
        return {
          x: matrix.e, y: matrix.f,
          top: matrix.f + box.y,
          left: matrix.e + box.x,
          right: matrix.e + box.x + box.width,
        };
      }

      const target = bounds(base);
      clients.forEach((path, index) => {
        const source = bounds(svg.querySelector(`g.node[id="S${index + 1}"]`));
        const fromLeft = source.x < target.x;
        const direction = fromLeft ? 1 : -1;
        // Separate the two links arriving at either side of the base router.
        const laneY = target.y + (index === 0 || index === 3 ? -8 : 8);
        const targetX = fromLeft ? target.left : target.right;
        const radius = Math.min(24, Math.abs(targetX - source.x) / 3);
        path.setAttribute('d',
          `M ${source.x},${source.top} ` +
          `L ${source.x},${laneY + radius} ` +
          `Q ${source.x},${laneY} ${source.x + direction * radius},${laneY} ` +
          `L ${targetX},${laneY}`);
      });
      svg.dataset.clientsRouted = 'true';
    });
  }

  let scheduled = false;
  function scheduleRouting() {
    if (scheduled) return;
    scheduled = true;
    requestAnimationFrame(() => {
      scheduled = false;
      routeTrackingClients();
    });
  }
  new MutationObserver(scheduleRouting).observe(document.documentElement, {
    childList: true,
    subtree: true,
  });
  scheduleRouting();
})();
