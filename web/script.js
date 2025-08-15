const filterBar = document.getElementById('filterBar');
const toggle = document.getElementById('toggleFilters');

// Toggle filter visibility
if (toggle && filterBar) {
  toggle.addEventListener('click', () => {
    const expanded = filterBar.classList.toggle('expanded');
    filterBar.classList.toggle('collapsed', !expanded);
    toggle.setAttribute('aria-expanded', expanded);
  });
}

// Apply theme based on browser preference
const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
function applyTheme(e) {
  document.documentElement.setAttribute('data-theme', e.matches ? 'dark' : 'light');
}
applyTheme(mediaQuery);
if (mediaQuery.addEventListener) {
  mediaQuery.addEventListener('change', applyTheme);
} else if (mediaQuery.addListener) { // Safari fallback
  mediaQuery.addListener(applyTheme);
}
